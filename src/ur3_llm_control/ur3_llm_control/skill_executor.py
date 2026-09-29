import argparse
import json
import threading
import time
from pathlib import Path
import rclpy
from rclpy.executors import MultiThreadedExecutor
from .robot_skills import RobotSkills
from .llm_planner import make_plan
from .task_validator import parse_plan, validate


def main():
    parser=argparse.ArgumentParser()
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--command',help='Natural-language command sent to 9Router')
    source.add_argument('--test-plan',help='Direct skill integration test, NOT an LLM run')
    source.add_argument('--inspect',action='store_true')
    parser.add_argument('--result',default='results/run.json')
    args=parser.parse_args()
    result_path=Path(args.result); result_path.parent.mkdir(parents=True,exist_ok=True)
    record={'mode':'llm' if args.command else 'skill_integration_test','command':args.command,'steps':[], 'success':False}
    rclpy.init(); node=RobotSkills(); executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    start=time.monotonic()
    try:
        node.ready()
        record['initial_positions']=node.positions()
        node.save_image(str(result_path.with_suffix('.before.png')))
        if args.inspect:
            record['mode']='inspection'; record['success']=True
        else:
            print('USER COMMAND: '+(args.command or '[DIRECT SKILL TEST]'),flush=True)
            if args.command:
                plan,record['llm']=make_plan(args.command,node.locations())
            else:
                plan=parse_plan(Path(args.test_plan).read_text())
            record['plan']=plan
            print('LLM PLAN:' if args.command else 'TEST PLAN:', json.dumps(plan),flush=True)
            steps=validate(plan,node.locations(),node.held)
            for step in steps:
                item=dict(step); begin=time.monotonic()
                try:
                    if step['skill']=='home': node.home()
                    elif step['skill']=='pick': node.pick(step['object'])
                    else: node.place(step['object'],step['zone'])
                    item['status']='SUCCESS'
                    time.sleep(0.6)
                    item['positions_after']=node.positions()
                    node.save_image(str(result_path.with_name(result_path.stem+f'.step{len(record["steps"])+1}.png')))
                except Exception as exc:
                    item['status']='FAILED'; item['error']=str(exc)
                    raise
                finally:
                    item['seconds']=round(time.monotonic()-begin,3); record['steps'].append(item)
                    print('EXECUTION: '+json.dumps(item),flush=True)
            record['success']=True
        record['final_positions']=node.positions()
    except Exception as exc:
        record['error']=str(exc)
        print('TASK FAILED: '+str(exc),flush=True)
        try: record['final_positions']=node.positions()
        except Exception: pass
    finally:
        record['motions']=node.motion_log; record['seconds']=round(time.monotonic()-start,3)
        record['held_at_end']=node.held
        record['image_saved']=node.save_image(str(result_path.with_suffix('.after.png')))
        result_path.write_text(json.dumps(record,ensure_ascii=False,indent=2))
        print('TASK SUCCESS' if record['success'] else 'TASK FAILED',flush=True)
        executor.shutdown(); thread.join(timeout=3); node.destroy_node(); rclpy.shutdown()
    return record
