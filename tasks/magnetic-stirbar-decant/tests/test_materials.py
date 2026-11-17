"""Independent physical invariants and meaningful fluid/contact counterexamples."""
import math
import numpy as np
from yam_arena import ik
from magnetic_decant import config as C
from magnetic_decant.materials import CylinderWater,MaterialModel,dipoles,capture
from magnetic_decant.motion import rotation


def object_state(position,R=np.eye(3)):
    return {'pose':[*position,*ik.matrix_to_quat_xyzw(R)],'velocity':[0.]*6}


def pouring_state(misaligned=False):
    R=rotation('Y',math.radians(-110));p=np.array([.30,0.,1.02]);shape=CylinderWater();lip=shape.lip(p,R)
    source=object_state(p,R);receiver=object_state([lip[0]-.012+(1 if misaligned else 0),0.,.80])
    bar=object_state(p+R@np.array([0,C.R_IN-C.BAR_RADIUS,.047]),R@rotation('Y',-math.pi/2))
    tool=object_state(p+R@np.array([0,.040,.047]),R@C.TOOL_SIDE)
    return {'objects':{'source':source,'receiver':receiver,'bar':bar,'restrainer':tool},'hands':{}}


def test_upright_free_surface_and_volume():
    shape=CylinderWater();p=np.array([0.,0.,.9]);h=shape.level(p,np.eye(3),C.VOLUME)
    assert abs(h-(p[2]+C.FLOOR+C.VOLUME/(math.pi*C.R_IN**2)))<1e-7
    assert abs(shape.volume(p,np.eye(3),h)-C.VOLUME)<1e-9


def test_water_inertia_remains_physical_at_tilt():
    shape=CylinderWater()
    for angle in (0,45,89,90,110,160):
        for v in (0.,.00007,.00015,.00028):
            _,mass,center,I=shape.properties(np.array([0.,0.,1.]),rotation('Y',math.radians(angle)),v)
            assert abs(mass-(C.DRY_MASS+1000*v))<.0007
            assert np.linalg.eigvalsh(I).min()>0
            assert C.FLOOR-1e-5<=center[2]<=C.HEIGHT


def test_magnetic_force_is_negative_energy_gradient():
    R1=rotation('X',.3);R2=rotation('Y',1.2);p1=np.array([0.,0.,0.]);p2=np.array([.014,.018,.007])
    force,t2,t1=dipoles(p1,R1,p2,R2)
    def energy(p):
        r=p-p1;s2=r@r+C.MAG_SOFTENING**2;m1=R1[:,2]*C.TOOL_MOMENT;m2=R2[:,0]*C.BAR_MOMENT
        return 1e-7*((m1@m2)/s2**1.5-3*(m1@r)*(m2@r)/s2**2.5)
    gradient=np.array([(energy(p2+np.eye(3)[k]*1e-7)-energy(p2-np.eye(3)[k]*1e-7))/2e-7 for k in range(3)])
    np.testing.assert_allclose(force,-gradient,rtol=1e-6,atol=1e-8)
    np.testing.assert_allclose(t1+t2+np.cross(p2-p1,force),0,atol=1e-14)


def test_coupling_falls_with_actual_separation():
    t=C.TOOL_SIDE;b=rotation('Z',math.pi/2)
    near=np.linalg.norm(dipoles(np.zeros(3),t,np.array([0.,-.012,0.]),b)[0])
    far=np.linalg.norm(dipoles(np.zeros(3),t,np.array([0.,-.080,0.]),b)[0])
    assert .05<near<5 and near>100*far


def test_end_on_wall_retention_is_valid_without_vertical_bar_assumption():
    s={'objects':{'source':object_state(C.SOURCE),'bar':object_state(C.SOURCE+[0,C.R_IN-C.BAR_LENGTH/2-.0005,.047],rotation('Z',math.pi/2)),
                  'restrainer':object_state(C.SOURCE+[0,.040,.047],C.TOOL_SIDE)}}
    assert capture(s)


def test_water_has_finite_flight_time():
    state=pouring_state();m=MaterialModel();assert capture(state)
    m.transport(state,.05)
    assert m.volumes['source']<C.VOLUME and m.volumes['receiver']==0 and m.packets
    assert abs(m.snapshot()['mass_balance_ml']-150)<1e-7


def test_actual_receiving_geometry_collects_and_conserves():
    state=pouring_state();m=MaterialModel()
    for _ in range(240):m.transport(state,.05)
    data=m.snapshot()
    assert data['receiver_ml']>140 and data['captured_delivered_ml']>140
    assert data['spilled_ml']<5
    assert abs(data['mass_balance_ml']-150)<1e-6


def test_missing_receiver_is_spill_not_teleport():
    state=pouring_state(True);m=MaterialModel()
    for _ in range(240):m.transport(state,.05)
    data=m.snapshot()
    assert data['receiver_ml']==0 and data['spilled_ml']>140
    assert abs(data['mass_balance_ml']-150)<1e-6


def test_ordinary_pour_without_restrainer_cannot_claim_magnetic_work():
    state=pouring_state();state['objects']['restrainer']=object_state([1.,1.,1.]);m=MaterialModel()
    for _ in range(240):m.transport(state,.05)
    assert m.volumes['receiver']>.00014 and m.captured_delivered==0


def test_solid_vessel_wall_blocks_a_water_packet():
    state=pouring_state();m=MaterialModel();p=np.asarray(state['objects']['receiver']['pose'][:3])
    hit=m._first_hit(p+[-.06,0,.04],p+[.06,0,.04],state)
    assert hit is not None and hit[1]=='spill' and hit[2]=='receiver'
