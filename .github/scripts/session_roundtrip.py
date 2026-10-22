import json, os, subprocess, sys, time

build, work = sys.argv[1], sys.argv[2]
src, out = os.path.join(work, "src"), os.path.join(work, "session")
os.makedirs(src)
W, H, N, STEP = 640, 480, 150, 2000000

dev = {"reg_info": {k: i + 1 for i, k in enumerate(
    "ax bx cx dx ay by cy dy dx_start dy_start dx_beta_start dy_beta_start dx_beta_inc dy_beta_inc "
    "dxdx_start dxdy_start dydx_start dydy_start dxdxdx_start dydxdx_start dxdxdy_start dydxdy_start "
    "dydydx_start dydydy_start".split())},
    "const_shift": 200.0}
dev["reg_info"]["pad_info"] = {"start_lines": 0, "end_lines": 0, "cropping_lines": 0}
dev["reg_info"]["zero_plane_info"] = {"dcmos_emitter_distance": 7.5, "dcmos_rcmos_distance": 2.3,
                                      "reference_distance": 120.0, "reference_pixel_size": 0.1042}
json.dump(dev, open(os.path.join(src, "device.json"), "w"))

frames = {}
t0 = time.time()
with open(os.path.join(src, "INDEX.txt"), "w") as idx:
    for i in range(N):
        t, ts = t0 + i / 30, (i + 1) * STEP
        d = (600 + i).to_bytes(2, "little") * (W * H)
        r = bytes([i % 256, (i * 7) % 256, (i * 13) % 256]) * (W * H)
        frames[("d", ts)], frames[("r", ts)] = d, r
        for typ, ext, hdr, body in (("d", "pgm", b"P5 640 480 65535\n", d), ("r", "ppm", b"P6 640 480 255\n", r)):
            name = f"{typ}-{t:f}-{ts}.{ext}"
            open(os.path.join(src, name), "wb").write(hdr + body)
            idx.write(name + "\n")

env = dict(os.environ, FAKENECT_PATH=src,
           LD_PRELOAD=next(os.path.join(dp, f) for dp, _, fs in os.walk(build) for f in fs
                           if f.startswith("libfakenect.so")))
p = subprocess.Popen([os.path.join(build, "bin", "freenect-session"), out], stdin=subprocess.PIPE,
                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, text=True)
for delay, cmd in ((1.0, "snap t1 20"), (1.5, "rec t2"), (1.0, "stop"), (0.5, "quit")):
    time.sleep(delay)
    p.stdin.write(cmd + "\n")
    p.stdin.flush()
log, _ = p.communicate(timeout=30)
print(log)

fail = []
def check(cond, msg):
    if not cond:
        fail.append(msg)

def read_capture(d):
    names = open(os.path.join(d, "INDEX.txt")).read().split()
    got = []
    for n in names:
        typ, ts = n[0], int(n.rsplit("-", 1)[1].split(".")[0])
        if typ == "a":
            continue
        body = open(os.path.join(d, n), "rb").read()
        body = body[body.index(b"\n") + 1:]
        check(frames.get((typ, ts)) == body, f"{d}/{n}: bytes differ from the source frame")
        got.append((typ, ts))
    return got

def contiguous(got, typ):
    ts = [t for k, t in got if k == typ]
    return ts == list(range(ts[0], ts[0] + STEP * len(ts), STEP)) if ts else False

t1 = read_capture(os.path.join(out, "t1"))
check(sum(k == "d" for k, _ in t1) == 20 and sum(k == "r" for k, _ in t1) == 20, f"t1 frame counts {len(t1)}")
check(contiguous(t1, "d") and contiguous(t1, "r"), "t1 skipped or reordered frames")
t2 = read_capture(os.path.join(out, "t2"))
check(sum(k == "d" for k, _ in t2) > 0 and contiguous(t2, "d") and contiguous(t2, "r"), "t2 empty or non-contiguous")
check(len(read_capture(os.path.join(out, "drift"))) >= 1, "no drift frame")

def flat(o, p=""):
    return {k2: v2 for k, v in o.items() for k2, v2 in (flat(v, p + k + ".").items() if isinstance(v, dict) else [(p + k, v)])}
for d in (out, os.path.join(out, "t1"), os.path.join(out, "t2")):
    a, b = flat(dev), flat(json.load(open(os.path.join(d, "device.json"))))
    check(all(abs(a[k] - b[k]) < 1e-4 for k in a), f"{d}/device.json does not round-trip")
check("dropped 0" in log and "gaps depth 0 rgb 0" in log, "drops or gaps reported")

print("\n".join(fail) or "round-trip OK")
sys.exit(1 if fail else 0)
