#!/usr/bin/env python3
"""cfgset.py - set values of the cfg of the controller from the command line, with a record.

  python3 tools/cfgset.py CFG KEY=VALUE [KEY=VALUE ...]     set; the old and new lines are printed
  python3 tools/cfgset.py CFG --show KEY [KEY ...]          print the current values

A KEY that occurs in one section only is given as is (BREAKAWAY_JOINT=3); one that occurs in several needs
its section (AXIS3.TORQUE_LIMIT_U=10). Only active lines are changed (not ';KEY=...'), inline comments are
kept, and a key that does not exist is an error (no line is added). Every change is appended to
CFG.changes.log with the time. The controller reads the cfg at its start: quit it ('q') and start it again
after a change. Standard library only.
The same as functions (values, plan, apply_files) for the tools that write measured values themselves
(jt_breakaway.py --apply, identify_rotor.py --apply, sine_cfg.py, kf_step.py): apply_files checks every item
against every file first and writes nothing if one fails.
"""
import argparse
import datetime
import os
import re
import sys

SEC = re.compile(r'\s*\[([^\]]+)\]')
KEY = re.compile(r'([ \t]*)([A-Za-z_][A-Za-z0-9_]*)([ \t]*=[ \t]*)([^;\r\n]*?)([ \t]*)(;[^\r\n]*)?(\r?\n)?$')
REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
CFG_MAIN = os.path.join(REPO, 'indy7', 'INDY7.cfg')                   # the cfg of the controller


def scan(lines):
    """{key: [(section, line index)]} of the active lines"""
    out, sec = {}, None
    for i, line in enumerate(lines):
        m = SEC.match(line)
        if m:
            sec = m.group(1).strip()
            continue
        m = KEY.match(line)
        if m:
            out.setdefault(m.group(2), []).append((sec, i))
    return out


def locate(index, name):
    """the line index of KEY or SECTION.KEY, or an error text"""
    if '.' in name:
        sec, key = name.split('.', 1)
    else:
        sec, key = None, name
    places = index.get(key, [])
    if sec is not None:
        places = [p for p in places if p[0] == sec]
    if not places:
        return None, 'no active line %s' % name
    if len(places) > 1:
        return None, '%s is in several sections (%s): give SECTION.%s' % (
            key, ', '.join(p[0] or '-' for p in places), key)
    return places[0][1], None


def read_lines(path):
    with open(path, encoding='utf-8', errors='surrogateescape', newline='') as f:
        return f.readlines()


def values(path, names):
    """{name: value text} of KEY or SECTION.KEY in path; a missing one is None"""
    lines = read_lines(path)
    index = scan(lines)
    out = {}
    for name in names:
        i, err = locate(index, name)
        out[name] = None if err else KEY.match(lines[i]).group(4)
    return out


def plan(path, items):
    """the changes that items (KEY=VALUE or (name, value)) make to path: (lines, changes, errors); nothing is
    written"""
    lines = read_lines(path)
    index = scan(lines)
    changes, errors = [], []
    for item in items:
        if isinstance(item, tuple):
            name, value = item
        elif '=' in item:
            name, value = item.split('=', 1)
        else:
            errors.append('%s: not KEY=VALUE' % item)
            continue
        name, value = name.strip(), str(value).strip()
        i, err = locate(index, name)
        if err:
            errors.append(err)
            continue
        m = KEY.match(lines[i])
        new = '%s%s%s%s%s%s%s' % (m.group(1), m.group(2), m.group(3), value, m.group(5) if m.group(6) else '',
                                  m.group(6) or '', m.group(7) or '\n')
        if new != lines[i]:
            changes.append((name, lines[i].rstrip('\r\n'), new.rstrip('\r\n')))
            lines[i] = new
    return lines, changes, errors


def commit(path, lines, changes, tag=None):
    with open(path, 'w', encoding='utf-8', errors='surrogateescape', newline='') as f:
        f.writelines(lines)
    stamp = datetime.datetime.now().strftime('%F %T')
    with open(path + '.changes.log', 'a', encoding='utf-8') as f:
        for name, old, new in changes:
            f.write('%s  %s%s: %s  ->  %s\n' % (stamp, ('[%s] ' % tag) if tag else '', name, old.strip(),
                                                  new.strip()))


def apply_files(paths, items, tag=None, prefix='  '):
    """set items in every file of paths, or in none: each is checked first. Prints the changed lines; returns
    (True, {path: changes}) or (False, errors)"""
    plans, errors = [], []
    for p in paths:
        try:
            lines, changes, errs = plan(p, items)
        except OSError as ex:
            errors.append('%s: %s' % (p, ex))
            continue
        errors += ['%s: %s' % (os.path.basename(p), e) for e in errs]
        plans.append((p, lines, changes))
    if errors:
        return False, errors
    done = {}
    for p, lines, changes in plans:
        if changes:
            commit(p, lines, changes, tag)
        done[p] = changes
        print('%s%s: %s' % (prefix, p, '%d change(s)' % len(changes) if changes else 'unchanged'))
        for name, old, new in changes:
            print('%s  %-28s %s  ->  %s' % (prefix, name, old.strip(), new.strip()))
    return True, done


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cfg')
    ap.add_argument('items', nargs='*', help='KEY=VALUE or SECTION.KEY=VALUE')
    ap.add_argument('--show', nargs='+', metavar='KEY', help='print the current values')
    a = ap.parse_args()
    if a.show:
        bad = 0
        for name, v in values(a.cfg, a.show).items():
            if v is None:
                lines = read_lines(a.cfg)
                print('cfgset: %s' % locate(scan(lines), name)[1])
                bad += 1
            else:
                print('%-28s %s' % (name, v))
        return 1 if bad else 0
    if not a.items:
        ap.error('KEY=VALUE or --show KEY')
    lines, changes, errors = plan(a.cfg, a.items)
    if errors:
        for e in errors:
            print('cfgset: %s' % e)
        print('cfgset: nothing written')
        return 1
    if not changes:
        print('cfgset: %s unchanged (the values are already set)' % a.cfg)
        return 0
    commit(a.cfg, lines, changes)
    for name, old, new in changes:
        print('  %-28s %s  ->  %s' % (name, old.strip(), new.strip()))
    print('cfgset: %d change(s) in %s (log: %s.changes.log); restart the controller' % (len(changes), a.cfg, a.cfg))
    return 0


if __name__ == '__main__':
    sys.exit(main())
