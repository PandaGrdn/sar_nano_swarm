from pathlib import Path
p = Path("/tmp/tri_fwd_phase0.log")
print("exists", p.is_file(), "size", p.stat().st_size if p.is_file() else 0)
if p.is_file():
    print(p.read_text(errors="replace"))
