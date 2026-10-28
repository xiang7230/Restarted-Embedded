from pathlib import Path
import sys, shutil
if len(sys.argv) != 2:
    raise SystemExit("Usage: python install_patch.py PATH_TO_RESTARTED_EMBEDDED")
repo = Path(sys.argv[1]).resolve()
base = Path(__file__).resolve().parent
main = repo / "app_program" / "app_main.c"
if not main.exists():
    raise SystemExit(f"Not a checkout: {main} missing")
source = main.read_text(encoding="utf-8")
old_init = '\n'.join('\tCHECK(subsystem%d_init(), "subsystem%d_init");' % (i,i) for i in range(1,6))
old_start = '\n'.join('\tCHECK(subsystem%d_start(), "subsystem%d_start");' % (i,i) for i in range(1,6))
if "robot_motion_init()" in source:
    raise SystemExit("Already installed")
if old_init not in source or old_start not in source:
    raise SystemExit("Source differs from reviewed version; no changes made")
for sub in ("subsystem2","subsystem3"):
    for file in (base/"app_program"/sub).iterdir():
        if (repo/"app_program"/sub/file.name).exists():
            raise SystemExit("File already exists: "+file.name)
source=source.replace('#include "app.h"', '#include "app.h"\n#include "subsystem2/robot_motion.h"\n#include "subsystem3/robot_line.h"',1)
source=source.replace(old_init, '''\tif (robot_motion_init() < E_OK || robot_line_init() < E_OK) {
\t\ttm_printf((UB*)"robot init failed\\n");
\t\ttk_slp_tsk(TMO_FEVR);
\t}''')
source=source.replace(old_start, '''\tif (robot_motion_start() < E_OK || robot_line_start() < E_OK) {
\t\ttm_printf((UB*)"robot task startup failed\\n");
\t\trobot_motion_fault();
\t\ttk_slp_tsk(TMO_FEVR);
\t}''')
a=source.find('#if APP_TEST\n\t/* Test build')
if a >= 0:
    b=source.index('#endif', a)+len('#endif')
    source=source[:a]+'#if APP_TEST\n\ttm_printf((UB*)"[TEST] example tests disabled for robot milestone\\n");\n#endif'+source[b:]
backup = main.with_suffix(".c.before_robot_patch")
shutil.copy2(main,backup)
for sub in ("subsystem2","subsystem3"):
    for file in (base/"app_program"/sub).iterdir():
        shutil.copy2(file, repo/"app_program"/sub/file.name)
main.write_text(source,encoding="utf-8")
print("Patched:",main)
print("Backup:",backup)
print("Motor output defaults to disabled. Verify wiring before enabling!")
