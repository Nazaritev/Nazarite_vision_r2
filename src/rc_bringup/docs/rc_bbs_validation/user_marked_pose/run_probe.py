#!/usr/bin/python3
"""Reproduce this offline diagnostic against the current already-built core."""
from pathlib import Path
import shlex,subprocess,os,sys
root=Path(__file__).resolve().parents[5]
folder=Path(__file__).resolve().parent
build=root/'build/rc_bbs_localization'
flags={}
for line in (build/'CMakeFiles/bbs_benchmark.dir/flags.make').read_text().splitlines():
 if ' = ' in line:
  k,v=line.split(' = ',1);flags[k]=shlex.split(v)
obj=Path('/tmp/rc-bbs-landmark-check/user_marked_probe.o')
obj.parent.mkdir(parents=True,exist_ok=True)
binary=obj.with_suffix('')
subprocess.run(['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',str(folder/'probe.cpp'),'-o',str(obj)],check=True)
link=shlex.split((build/'CMakeFiles/bbs_benchmark.dir/link.txt').read_text())
link[link.index('CMakeFiles/bbs_benchmark.dir/src/benchmark.cpp.o')]=str(obj)
link[link.index('-o')+1]=str(binary)
subprocess.run(link,cwd=build,check=True)
config=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else folder/'parameters.yaml'
subprocess.run([str(binary),str(config)],env=dict(os.environ,OMP_NUM_THREADS='4',OMP_WAIT_POLICY='PASSIVE'),check=True)
