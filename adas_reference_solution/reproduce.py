"""One command: tests -> experiments -> Turkish PDF and Markdown -> manifest."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

ROOT=Path(__file__).resolve().parent


def main():
    data=ROOT/'output'/'data'; data.mkdir(parents=True,exist_ok=True)
    test=subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-v'],
                        cwd=ROOT,capture_output=True,text=True)
    (data/'test_results.txt').write_text(test.stdout+test.stderr,encoding='utf-8')
    print(test.stdout+test.stderr)
    if test.returncode:
        raise SystemExit(test.returncode)
    for script in ('run_all.py','build_report.py'):
        subprocess.run([sys.executable,str(ROOT/script)],cwd=ROOT,check=True)
    files=sorted([*ROOT.glob('*.py'),*ROOT.glob('adas/*.py'),*ROOT.glob('tests/*.py'),
                  ROOT/'requirements.txt',data/'results.json',data/'test_results.txt'])
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (data/'manifest.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    print('Complete:',ROOT/'output'/'pdf'/'adas_referans_raporu.pdf')


if __name__=='__main__': main()
