import argparse
import json
import os
import subprocess
from pathlib import Path
import urllib.request
import yaml
from ament_index_python.packages import get_package_share_directory
from .task_validator import parse_plan, validate, assignment


def make_plan(command, locations=None):
    root = Path(get_package_share_directory('ur3_llm_control'))
    student = yaml.safe_load((root/'config/student_config.yaml').read_text())
    mapping = assignment(student['student_id'])[1] if student.get('student_id') else None
    model = os.environ.get('NINEROUTER_MODEL', '')
    if not model:
        raise RuntimeError('Set NINEROUTER_MODEL to a model connected in 9Router')
    base = os.environ.get('NINEROUTER_BASE_URL', 'http://127.0.0.1:20128/v1').rstrip('/')
    context = {'student_assignment': mapping, 'locations': locations or {}}
    payload = {'model': model, 'temperature': 0, 'stream': False, 'max_tokens': 1500,
               'messages': [{'role':'system','content':(root/'prompts/planner.txt').read_text()},
                            {'role':'system','content':json.dumps(context)},
                            {'role':'user','content':command}]}
    headers = {'Content-Type':'application/json'}
    key = os.environ.get('NINEROUTER_API_KEY')
    if key:
        headers['Authorization'] = 'Bearer ' + key
    req = urllib.request.Request(base+'/chat/completions', data=json.dumps(payload).encode(), headers=headers)
    bridge = os.environ.get('NINEROUTER_WINDOWS_BRIDGE')
    if bridge:
        proc = subprocess.run([os.environ.get('WINDOWS_NODE', '/mnt/c/Program Files/nodejs/node.exe'), bridge],
                              input=json.dumps({'url':base+'/chat/completions','headers':headers,'payload':payload}),
                              text=True, capture_output=True, timeout=100)
        if proc.returncode:
            raise RuntimeError(proc.stderr[:1500])
        data = json.loads(proc.stdout)
    else:
        with urllib.request.urlopen(req, timeout=90) as response:
            data = json.load(response)
    raw = data['choices'][0]['message']['content']
    if not isinstance(raw, str):
        raise ValueError('LLM did not return text JSON')
    plan = parse_plan(raw)
    return plan, {'model':model, 'usage': data.get('usage', {}), 'raw_plan':raw}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command')
    parser.add_argument('--output', default='plan.json')
    args = parser.parse_args()
    plan, meta = make_plan(args.command)
    validate(plan)
    Path(args.output).write_text(json.dumps({'command':args.command,'plan':plan['plan'],'llm':meta}, ensure_ascii=False, indent=2))
    print(json.dumps(plan, ensure_ascii=False, indent=2))
