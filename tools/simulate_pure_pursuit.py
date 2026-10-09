"""Offline 50 ms PP simulations; never opens hardware."""
from pathlib import Path
import sys
import json
import math
from dataclasses import replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'code'))
from components.differential_navigation import DifferentialPathController, NavigationState
from config.v2_loader import load_v2_config
from core.types import Pose2D


def simulate(start,end,pose,*,reverse=False,disturb=False):
    config=load_v2_config()
    controller=DifferentialPathController(replace(config.navigation,lookahead_m=.20,position_tolerance_m=.005),
        replace(config.drive,max_linear_speed_m_s=.15),track_width_m=.198)
    x,y,yaw=pose
    progress=0.
    speeds=[]
    for i in range(8000):
        result=controller.compute_line(Pose2D(x,y,yaw,i*.05),start,end,progress,reverse=reverse)
        progress=result.diagnostics['progress_m']
        v,w=result.command.linear_x_m_s,result.command.angular_z_rad_s
        if result.state is NavigationState.BLOCKED:
            raise AssertionError(result.diagnostics)
        if result.state is NavigationState.GOAL_REACHED:
            break
        assert v <= 0 if reverse else v >= 0
        if v:
            assert (v-w*.198/2)*(v+w*.198/2) >= 0
        speeds.append(v)
        heading=yaw+w*.025
        x+=v*math.cos(heading)*.05
        y+=v*math.sin(heading)*.05
        yaw+=w*.05
        if disturb and i < 10:
            yaw+=math.radians(.5) # 0.5 s post-turn wheel/yaw disturbance
    else:
        raise AssertionError('simulation timeout')
    return {'steps':i,'elapsed_s':round(i*.05,3),'final_pose':[x,y,yaw],
            'raw_progress_m':result.diagnostics['raw_progress_m'],
            'cross_track_m':result.diagnostics['cross_track_m'],
            'min_v_m_s':min(speeds,default=0),'max_v_m_s':max(speeds,default=0),
            'status':result.diagnostics['line_state']}


def main():
    results={}
    for y in (-.08,.08):
        results[f'2.8m_initial_y_{y:+.2f}']=simulate((0,0),(2.8,0),(0,y,0))
    outbound=simulate((0,0),(.47,0),(0,0,0),disturb=True)
    results['47cm_post_turn_disturbance']=outbound
    results['47cm_reverse_original_endpoints']=simulate((.47,0),(0,0),outbound['final_pose'],reverse=True)
    print(json.dumps({'scope':'offline software simulation only','dt_s':.05,'physical_track_m':.198,
                      'firmware_protocol_track_m':.164,'cases':results},indent=2))

if __name__=='__main__': main()
