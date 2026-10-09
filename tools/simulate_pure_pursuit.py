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


def simulate(start,end,pose,*,reverse=False,disturb=False,coast=False):
    config=load_v2_config()
    controller=DifferentialPathController(replace(config.navigation,lookahead_m=.20,position_tolerance_m=.005),
        replace(config.drive,max_linear_speed_m_s=.15),track_width_m=.198)
    x,y,yaw=pose
    progress=0.
    speeds=[]
    align_ticks=settled_ticks=0
    coast_frames=0
    first_translation_step=None
    settled_stable_frames=None
    for i in range(8000):
        p=Pose2D(x,y,yaw,i*.05)
        result=controller.compute_line(p,start,end,progress,reverse=reverse,t265_pose=p,now_s=i*.05)
        progress=result.diagnostics['progress_m']
        v,w=result.command.linear_x_m_s,result.command.angular_z_rad_s
        if result.state in {NavigationState.BLOCKED, NavigationState.POSE_LOST}:
            raise AssertionError(result.diagnostics)
        if result.state is NavigationState.GOAL_REACHED:
            break
        if result.state is NavigationState.ROTATING_TO_PATH:
            align_ticks+=1
            assert v == 0
            if result.diagnostics['line_state'] == 'align_settled':
                settled_ticks+=1
                settled_stable_frames=result.diagnostics['stable_frames']
                assert w == 0 and settled_stable_frames >= 4
        if v and first_translation_step is None:
            first_translation_step=i
            if coast:
                assert settled_ticks == 1 and coast_frames == 6
        assert v <= 0 if reverse else v >= 0
        if v:
            assert (v-w*.198/2)*(v+w*.198/2) >= 0
        speeds.append(v)
        heading=yaw+w*.025
        x+=v*math.cos(heading)*.05
        y+=v*math.sin(heading)*.05
        yaw+=w*.05
        if (coast and coast_frames < 6 and result.diagnostics['line_state'] == 'align_forward'
                and abs(result.diagnostics['yaw_error_rad']) <= math.radians(3)):
            yaw+=math.radians(.5) # Real post-command yaw continues for six 50 ms samples.
            coast_frames+=1
        if disturb and i < 10:
            yaw+=math.radians(.5) # 0.5 s post-turn wheel/yaw disturbance
    else:
        raise AssertionError('simulation timeout')
    return {'steps':i,'elapsed_s':round(i*.05,3),'final_pose':[x,y,yaw],
            'raw_progress_m':result.diagnostics['raw_progress_m'],
            'cross_track_m':result.diagnostics['cross_track_m'],
            'min_v_m_s':min(speeds,default=0),'max_v_m_s':max(speeds,default=0),
            'status':result.diagnostics['line_state'], 'alignment_ticks':align_ticks,
            'align_settled_ticks':settled_ticks,'injected_coast_frames':coast_frames,
            'first_translation_step':first_translation_step,'stable_frames_at_settle':settled_stable_frames}


def main():
    results={}
    for y in (-.08,.08):
        results[f'2.8m_initial_y_{y:+.2f}']=simulate((0,0),(2.8,0),(0,y,0))
    outbound=simulate((0,0),(.47,0),(0,0,0),disturb=True)
    results['47cm_post_turn_disturbance']=outbound
    results['47cm_reverse_original_endpoints']=simulate((.47,0),(0,0),outbound['final_pose'],reverse=True)
    results['47cm_endpoint_7mm_remaining_2cm_lateral']=simulate((0,0),(.47,0),(.463,.02,0))
    results['large_angle_alignment_with_measured_coast']=simulate((0,0),(2.8,0),(0,0,math.pi/2),coast=True)
    print(json.dumps({'scope':'offline software simulation only','dt_s':.05,'physical_track_m':.198,
                      'firmware_protocol_track_m':.164,'cases':results},indent=2))

if __name__=='__main__': main()
