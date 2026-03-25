import shutil
import os

src = '/home/kenneth_m/ARHA_ws/src/ARHA/pincopen/meshes'
dst = '/home/kenneth_m/ARHA_ws/src/ARHA/arha_description/meshes/pincopen'

if not os.path.exists(dst):
    os.makedirs(dst)

for f in os.listdir(src):
    shutil.copy(os.path.join(src, f), os.path.join(dst, f))

print("Copied meshes successfully.")
