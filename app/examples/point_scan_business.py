"""Embedding example; import performs no connection, motion or acquisition.

The business UI supplies selected_routes and chooses a slot. Its own capture
callback supplies (acquisition_id, passed). Run blocking methods in a worker.
"""
from app.point_bank import upload_frames


def prepare_selected_routes(controller, selected_routes):
    """Validate all chosen routes before the first upload; then preload 1..6."""
    selected_routes=tuple(selected_routes)
    if not 1<=len(selected_routes)<=6:raise ValueError('选择1～6套路线')
    plans=[]
    for slot,item in enumerate(selected_routes,1):
        points=tuple(tuple(p) for p in item['points'])
        name=item['name'];speed=item.get('speed_rpm',.1);tolerance=item.get('tolerance_deg',.2)
        if not isinstance(name,str):raise ValueError('路线name须为字符串')
        upload_frames(points,slot=slot,token=1,speed_rpm=speed,tolerance_deg=tolerance)
        plans.append((slot,name,points,speed,tolerance))
    # Uploads are sequential and not atomic across banks. On any failure,
    # report it; no RUN is sent and no automatic retry is attempted.
    return {slot:controller.prepare_point_scan(points,name=name,route_id=slot,
                speed_rpm=speed,tolerance_deg=tolerance)
            for slot,name,points,speed,tolerance in plans}


def run_until_decision_holds(scan, acquire_and_judge, *, timeout_s=120):
    """Advance only on passed acquisition; return held scan for caller decisions.

    Callback signature: (point_index, targets_deg, completed_positions) ->
    (nonempty_acquisition_id, bool_passed). Laser code belongs to the caller.
    """
    while scan.status()['can_start_next']:
        done=scan.move_next_and_wait(timeout_s=timeout_s)
        acquisition_id,passed=acquire_and_judge(done['point_index'],done['targets_deg'],done['positions'])
        state=scan.confirm_acquisition(done['point_index'],acquisition_id,passed=passed)
        if not passed or state['phase']=='COMPLETED':return state
    return scan.status()
