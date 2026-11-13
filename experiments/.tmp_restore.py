import os

BAK = '/tmp/b2_wip_backup/files/'
ROOT = '/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/'


def rd(p):
    return open(p, encoding='utf-8').read().replace('\r\n', '\n').split('\n')


def extract(lines, start_marker, end_marker):
    si = [i for i, l in enumerate(lines) if start_marker in l]
    assert len(si) == 1, ('start marker', start_marker, len(si))
    s = si[0]
    ei = [i for i in range(s, len(lines)) if end_marker in lines[i]]
    assert ei, ('end marker', end_marker)
    return lines[s:ei[0] + 1]


def insert_after(lines, anchor, block):
    idx = [i for i, l in enumerate(lines) if l.strip() == anchor]
    assert len(idx) == 1, ('anchor', anchor, len(idx))
    i = idx[0]
    return lines[:i + 1] + [''] + block + lines[i + 1:]


def write(p, lines):
    open(p, 'w', encoding='utf-8', newline='\n').write('\n'.join(lines))


jobs = [
    # (backup index, target path, start marker, end marker, anchor line)
    ('5', ROOT + 'include/lio/params.h',
     '===== B2：视觉相对旋转约束', 'extern bool   g_visual_b3_fd_check;',
     'extern bool g_fast_tf;'),
    ('6', ROOT + 'src/lio/params.cpp',
     '===== B2：视觉相对旋转约束', 'bool   g_visual_b3_fd_check = true;',
     'bool g_fast_tf = false;'),
    ('7', ROOT + 'src/ros/ROSWrapper.cpp',
     '===== B2：视觉相对旋转（光流两帧几何', '<< " topic=" << g_visual_image_topic << RESET;',
     'node.get_parameter("lio.submap.submap_capacity", g_submap_capacity);'),
]

for bi, target, sm, em, anchor in jobs:
    block = extract(rd(BAK + bi), sm, em)
    cur = rd(target)
    assert not any('g_visual_enable' in l for l in cur), ('already patched', target)
    out = insert_after(cur, anchor, block)
    write(target, out)
    print('patched', target, 'block lines =', len(block))
print('done')
