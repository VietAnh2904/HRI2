"""Strict syntax and state-machine validation. Never evaluates generated code."""
import json
import re

OBJECTS = ('red_cube', 'yellow_cube', 'blue_cube')
ZONES = ('zone_a', 'zone_b', 'zone_c')
PERMUTATIONS = ((0,1,2), (0,2,1), (1,0,2), (1,2,0), (2,0,1), (2,1,0))


class PlanError(ValueError):
    pass


def assignment(student_id):
    if not isinstance(student_id, str) or not re.fullmatch(r'\d{2,}', student_id, flags=re.ASCII):
        raise PlanError('STUDENT_ID_REQUIRED: configure your real student ID')
    p = int(student_id[-2:]) % 6
    return p, dict(zip(ZONES, (OBJECTS[i] for i in PERMUTATIONS[p])))


def _unique_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise PlanError('DUPLICATE_JSON_KEY')
        value[key] = item
    return value


def parse_plan(raw):
    if len(raw) > 16384:
        raise PlanError('PLAN_TOO_LARGE')
    try:
        return json.loads(raw, object_pairs_hook=_unique_object)
    except (json.JSONDecodeError, RecursionError) as exc:
        raise PlanError('INVALID_JSON') from exc


def validate(plan, locations=None, held=None):
    if not isinstance(plan, dict) or set(plan) != {'plan'}:
        raise PlanError('INVALID_ROOT_SCHEMA')
    steps = plan['plan']
    if not isinstance(steps, list) or not 1 <= len(steps) <= 25:
        raise PlanError('INVALID_PLAN_LENGTH')
    state = dict(locations or {o: None for o in OBJECTS})
    fields = {'home': {'skill'}, 'pick': {'skill','object'}, 'place': {'skill','object','zone'}}
    for index, step in enumerate(steps):
        if not isinstance(step, dict) or not isinstance(step.get('skill'), str) or step['skill'] not in fields:
            raise PlanError(f'INVALID_SKILL at {index}')
        skill = step['skill']
        if set(step) != fields[skill]:
            raise PlanError(f'INVALID_ARGUMENTS at {index}')
        obj = step.get('object')
        if skill != 'home' and obj not in OBJECTS:
            raise PlanError(f'INVALID_OBJECT at {index}')
        if skill == 'pick':
            if held is not None:
                raise PlanError(f'GRIPPER_OCCUPIED at {index}')
            held = obj
            state[obj] = None
        elif skill == 'place':
            zone = step['zone']
            if zone not in ZONES:
                raise PlanError(f'INVALID_ZONE at {index}')
            if held != obj:
                raise PlanError(f'OBJECT_NOT_HELD at {index}')
            if zone in state.values():
                raise PlanError(f'ZONE_OCCUPIED at {index}')
            state[obj] = zone
            held = None
        elif held is not None:
            raise PlanError(f'HOME_WHILE_HOLDING at {index}')
    if held is not None or steps[-1]['skill'] != 'home':
        raise PlanError('PLAN_MUST_FINISH_EMPTY_AT_HOME')
    return steps
