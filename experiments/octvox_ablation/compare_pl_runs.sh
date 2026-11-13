#!/usr/bin/env bash
echo "=== fork run stats ==="
grep -a "彩色统计" /home/ywj/slam_data/colored_lio_20261110/pointlio/vis.log | tail -1
echo "=== SMBU run stats ==="
grep -a "彩色统计" /home/ywj/slam_data/colored_lio_20261110/pointlio_smbu/vis.log | tail -1
echo "=== PCD extent comparison (python) ==="
python3 - <<'EOF'
import numpy as np
def load_xyz(path):
    with open(path,'rb') as f:
        header={}
        while True:
            line=f.readline().decode('ascii').strip()
            if line.startswith('DATA'):
                header['data']=line.split()[1]; break
            p=line.split()
            if p: header[p[0]]=p[1:]
        fields=header['FIELDS']; n=int(header['POINTS'][0])
        types={'float':4,'int':4,'uchar':1,'uint':4}
        sizes=[int(s) for s in header['SIZE']]
        dtypes={'F':'<f4','I':'<i4','U':'<u1'}
        dtype=np.dtype({'names':fields,'formats':[dtypes[t] for t in header['TYPE']],'offsets':np.cumsum([0]+sizes[:-1]),'itemsize':sum(sizes)})
        if header['data']=='binary':
            arr=np.frombuffer(f.read(n*sum(sizes)),dtype=dtype)
        else:
            arr=np.loadtxt(path,skiprows=len(open(path).readlines()),max_rows=n,dtype=dtype)
    return np.stack([arr['x'],arr['y'],arr['z']],axis=1).astype(np.float64)
for name in ['fastlio2','pointlio','pointlio_smbu']:
    p=f'/home/ywj/slam_data/colored_lio_20261110/{name}/super_livo_visual_a_colored.pcd'
    try:
        xyz=load_xyz(p)
        mn=xyz.min(0); mx=xyz.max(0)
        print(f'{name:15s} n={len(xyz):8d}  x[{mn[0]:8.1f},{mx[0]:8.1f}] y[{mn[1]:8.1f},{mx[1]:8.1f}] z[{mn[2]:7.1f},{mx[2]:7.1f}]  span=({mx[0]-mn[0]:7.1f} x {mx[1]-mn[1]:7.1f} x {mx[2]-mn[2]:6.1f})')
    except Exception as e:
        print(name,'ERR',e)
EOF
