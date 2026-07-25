#!/usr/bin/env python3
import argparse, os, select, signal, subprocess, sys, termios, threading, time, tty
from datetime import datetime
from pathlib import Path

BANNER='''========================================\nINNO RPLIDAR SLAM\ns : 현재 지도 저장\nq : SLAM 종료\nh : 도움말\n========================================'''
class Runner:
    def __init__(self, launch_args): self.launch_args=launch_args; self.proc=None; self.saving=False; self.old=None
    def unique_base(self):
        maps=Path.home()/'inno_jazzy_ws'/'maps'; maps.mkdir(parents=True,exist_ok=True)
        stem='inno_map_'+datetime.now().strftime('%Y%m%d_%H%M%S'); base=maps/stem; n=1
        while base.with_suffix('.yaml').exists() or base.with_suffix('.pgm').exists(): base=maps/f'{stem}_{n:02d}'; n+=1
        return base
    def save(self):
        if self.saving: print('\n[지도 저장] 이미 저장 중이므로 입력을 무시합니다.',flush=True); return
        self.saving=True
        def work():
            base=self.unique_base(); print(f'\n[지도 저장] 시작: {base}',flush=True)
            try:
                check=subprocess.run(['timeout','6','ros2','topic','echo','/map','--once'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
                if check.returncode: raise RuntimeError('/map 메시지를 6초 안에 받지 못했습니다: '+check.stderr.strip())
                save=subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-t','/map','-f',str(base)],text=True)
                yaml=base.with_suffix('.yaml'); pgm=base.with_suffix('.pgm')
                if save.returncode or not yaml.is_file() or not pgm.is_file(): raise RuntimeError(f'map_saver_cli 실패(code={save.returncode}) 또는 결과 파일 없음')
                for link,target in ((base.parent/'latest_map.yaml',yaml),(base.parent/'latest_map.pgm',pgm)):
                    try: link.unlink(missing_ok=True); link.symlink_to(target.name)
                    except OSError as e: print(f'[경고] latest 링크 생성 실패: {e}',flush=True)
                print(f'[지도 저장 성공]\n{yaml}\n{pgm}',flush=True)
                self.posegraph(base)
            except Exception as e: print(f'[지도 저장 실패] {e}\nSLAM은 계속 실행됩니다.',file=sys.stderr,flush=True)
            finally: self.saving=False
        threading.Thread(target=work,daemon=True).start()
    def posegraph(self,base):
        listing=subprocess.run(['ros2','service','list','-t'],capture_output=True,text=True)
        service=next((line.split()[0] for line in listing.stdout.splitlines() if 'slam_toolbox/srv/SerializePoseGraph' in line),None)
        if not service: print('[pose graph] serialize 서비스가 없어 건너뜁니다.',flush=True); return
        graph=str(base).replace('inno_map_','inno_posegraph_')
        call=subprocess.run(['ros2','service','call',service,'slam_toolbox/srv/SerializePoseGraph',f'{{filename: "{graph}"}}'],text=True)
        print(('[pose graph 저장 요청 성공] ' if call.returncode==0 else '[pose graph 저장 실패] ')+graph,flush=True)
    def stop(self):
        if self.proc and self.proc.poll() is None:
            print('\n[종료] launch 프로세스 그룹에 SIGINT를 전달합니다.',flush=True); os.killpg(self.proc.pid,signal.SIGINT)
            try: self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                print('[종료] 15초 초과, SIGTERM을 전달합니다.',flush=True); os.killpg(self.proc.pid,signal.SIGTERM); self.proc.wait(timeout=5)
    def run(self):
        if not sys.stdin.isatty(): print('오류: 키 입력용 현재 터미널이 TTY가 아닙니다.',file=sys.stderr); return 2
        self.proc=subprocess.Popen(['ros2','launch','inno_robot_bringup','slam_full.launch.py']+self.launch_args,start_new_session=True)
        self.old=termios.tcgetattr(sys.stdin.fileno()); tty.setcbreak(sys.stdin.fileno()); print(BANNER,flush=True)
        try:
            while self.proc.poll() is None:
                ready,_,_=select.select([sys.stdin],[],[],0.2)
                if ready:
                    ch=sys.stdin.read(1)
                    if ch in 'sS': self.save()
                    elif ch in 'hH': print('\n'+BANNER,flush=True)
                    elif ch in 'qQ': self.stop(); break
        except KeyboardInterrupt: self.stop()
        finally:
            if self.old: termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,self.old)
            self.stop()
        return self.proc.returncode or 0
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('launch_args',nargs='*'); ns=ap.parse_args(); raise SystemExit(Runner(ns.launch_args).run())
