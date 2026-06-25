python - <<'PY'
from pathlib import Path
import math

src = Path("scene/assets/meshes/teleop_station_visualizeA2.obj")
dst = Path("scene/assets/meshes/teleop_clean2.obj")

bad_vertices = set()
vertex_index = 0 

with src.open('r', errors='ignore') as f, dst.open('w') as out:
    for line in f:
        if line.startswith("v "):
            vertex_index += 1
            parts = line.split()
            try:
                coords = [float(parts[1]), float(parts[2]), float(parts[3])]

                if all(math.isfinite(c) for c in coords):
                    out.write(line)
                else:
                    bad_vertices.append(vertex_index)
                    out.write("v 0 0 0\n")

            except Exception:
                bad_vertices.append(vertex_index)
                out.write("v 0 0 0\n")
        else:
            out.write(line)
print(f"Cleaned OBJ written to: {dst}")
print(f"Bad vertices replaced: {len(bad_vertices)}")
print(f"Bad vertex indices: {sorted(bad_vertices)[:20]}")
PY