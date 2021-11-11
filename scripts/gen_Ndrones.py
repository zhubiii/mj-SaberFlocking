"""
File to generate N drones into an xml file. Each drone
to have unique indentifiers and randomly generated positions
"""
from xml.dom import minidom
import os
import sys
import random

SPAWN_RADIUS    = 5 # maximum spawn distance of drones
# list to store the unique drone xml elements
drones          = []
geom_core       = []
geom_a0         = []
geom_a1         = []
geom_a2         = []
geom_a3         = []
joint           = []
site_m0         = []
site_m1         = []
site_m2         = []
site_m3         = []
geom_thrust0    = []
geom_thrust1    = []
geom_thrust2    = []
geom_thrust3    = []
site_qcX        = []
site_qcY        = []
site_qcZ        = []
actuator        = []
act_m0          = []
act_m1          = []
act_m2          = []
act_m3          = []

if __name__ == "__main__":
    # Ensure that a number is passed in as an arg
    if len(sys.argv) < 2 or not sys.argv[1].isnumeric():
        print("\nUSAGE:   python3 gen_Ndrones.py [# of drones]\n")
        exit(1)

    N = sys.argv[1]
    filename = "model/"+N+"drones.xml"
    N = int(N)
    root = minidom.Document()
  
    xml = root.createElement('mujoco') 
    root.appendChild(xml)
  
    compiler = root.createElement('compiler')
    compiler.setAttribute('inertiafromgeom', 'true')
    compiler.setAttribute('coordinate', 'local')
    compiler.setAttribute('angle', 'radian')

    option = root.createElement('option')
    option.setAttribute('timestep', '0.01')
    option.setAttribute('gravity', '0 0 -9.81')
    option.setAttribute('density', '1.2')
    option.setAttribute('viscosity', '0.00002')

    asset = root.createElement('asset')
    texture1 = root.createElement('texture')
    texture1.setAttribute('type', 'skybox')
    texture1.setAttribute('builtin', 'gradient')
    texture1.setAttribute('rgb1', '0.3 0.5 0.7')
    texture1.setAttribute('rgb2', '0.0 0.0 0.0')
    texture1.setAttribute('width', '512')
    texture1.setAttribute('height', '512')
    texture2 = root.createElement('texture')
    texture2.setAttribute('name', 'texplane')
    texture2.setAttribute('type', '2d')
    texture2.setAttribute('builtin', 'checker')
    texture2.setAttribute('rgb1', '0.3 0.5 0.7')
    texture2.setAttribute('rgb2', '0.1 0.15 0.2')
    texture2.setAttribute('width', '512')
    texture2.setAttribute('height', '512')
    texture3 = root.createElement('texture')
    texture3.setAttribute('name', 'texgeom')
    texture3.setAttribute('type', 'cube')
    texture3.setAttribute('builtin', 'flat')
    texture3.setAttribute('mark', 'cross')
    texture3.setAttribute('width', '127')
    texture3.setAttribute('height', '1278')
    material1 = root.createElement('material')
    material1.setAttribute('name', 'matplane')
    material1.setAttribute('reflectance', '0.3')
    material1.setAttribute('texture', 'texplane')
    material1.setAttribute('texrepeat', '1 1')
    material1.setAttribute('texuniform', 'true')
    material2 = root.createElement('material')
    material2.setAttribute('name', 'matgeom')
    material2.setAttribute('texture', 'texgeom')
    material2.setAttribute('texuniform', 'true')
    material2.setAttribute('rgba', '0.8 0.6 0.4 1')

    asset.appendChild(texture1)
    asset.appendChild(texture2)
    asset.appendChild(texture3)
    asset.appendChild(material1)
    asset.appendChild(material2)

    worldbody = root.createElement('worldbody')
    geom = root.createElement('geom')
    geom.setAttribute('name', 'floor')
    geom.setAttribute('pos', '0 0 0')
    geom.setAttribute('size', '0 0 0.25')
    geom.setAttribute('type', 'plane')
    geom.setAttribute('material', 'matplane')
    geom.setAttribute('condim', '3')
    light1 = root.createElement('light')
    light1.setAttribute('directional', 'false')
    light1.setAttribute('diffuse', '0.2 0.2 0.2')
    light1.setAttribute('specular', '0.0 0.0 0.0')
    light1.setAttribute('pos', '0.0 0.0 5.0')
    light1.setAttribute('dir', '0.0 0.0 -1.0')
    light1.setAttribute('castshadow', 'false')
    worldbody.appendChild(geom)
    worldbody.appendChild(light1)
    for i in range(N):
        drones.append(root.createElement('body'))
        drones[i].setAttribute('name', 'quadrotor'+str(i))
        drones[i].setAttribute(
            'pos',
            str(random.uniform(-1,1)*SPAWN_RADIUS)
            +' '+str(random.uniform(-1, 1)*SPAWN_RADIUS)
            +' '+str(0)
        )
        geom_core.append(root.createElement('geom'))
        geom_core[i].setAttribute('name', 'core'+str(i))
        geom_core[i].setAttribute('type', 'box')
        geom_core[i].setAttribute('pos', '0 0 0')
        geom_core[i].setAttribute('quat', '1 0 0 0')
        geom_core[i].setAttribute('size', '0.06 0.035 0.025')
        geom_core[i].setAttribute('rgba', '0.3 0.3 0.8 1')
        geom_core[i].setAttribute('mass', '0.4')
        drones[i].appendChild(geom_core[i])

        geom_a0.append(root.createElement('geom'))
        geom_a0[i].setAttribute('name', 'a00'+str(i))
        geom_a0[i].setAttribute('type', 'box')
        geom_a0[i].setAttribute('pos', '0.071 0.071 0.0')
        geom_a0[i].setAttribute('size', '0.05 0.01 0.0025')
        geom_a0[i].setAttribute('quat', '0.924 0.0 0.0 0.383')
        geom_a0[i].setAttribute('rgba', '0.3 0.3 0.3 1')
        geom_a0[i].setAttribute('mass', '0.025')
        drones[i].appendChild(geom_a0[i])

        geom_a1.append(root.createElement('geom'))
        geom_a1[i].setAttribute('name', 'a10'+str(i))
        geom_a1[i].setAttribute('type', 'box')
        geom_a1[i].setAttribute('pos', '0.071 -0.071 0.0')
        geom_a1[i].setAttribute('size', '0.05 0.01 0.0025')
        geom_a1[i].setAttribute('quat', '0.383 0.0 0.0 0.924')
        geom_a1[i].setAttribute('rgba', '0.3 0.3 0.3 1')
        geom_a1[i].setAttribute('mass', '0.025')
        drones[i].appendChild(geom_a1[i])

        geom_a2.append(root.createElement('geom'))
        geom_a2[i].setAttribute('name', 'a20'+str(i))
        geom_a2[i].setAttribute('type', 'box')
        geom_a2[i].setAttribute('pos', '-0.071 -0.071 0.0')
        geom_a2[i].setAttribute('size', '0.05 0.01 0.0025')
        geom_a2[i].setAttribute('quat', '-0.383 0.0 0.0 0.924')
        geom_a2[i].setAttribute('rgba', '0.3 0.3 0.3 1')
        geom_a2[i].setAttribute('mass', '0.025')
        drones[i].appendChild(geom_a2[i])

        geom_a3.append(root.createElement('geom'))
        geom_a3[i].setAttribute('name', 'a30'+str(i))
        geom_a3[i].setAttribute('type', 'box')
        geom_a3[i].setAttribute('pos', '-0.071 0.071 0.0')
        geom_a3[i].setAttribute('size', '0.05 0.01 0.0025')
        geom_a3[i].setAttribute('quat', '0.924 0.0 0.0 -0.383')
        geom_a3[i].setAttribute('rgba', '0.3 0.3 0.3 1')
        geom_a3[i].setAttribute('mass', '0.025')
        drones[i].appendChild(geom_a3[i])

        joint.append(root.createElement('joint'))
        joint[i].setAttribute('name', 'root'+str(i))
        joint[i].setAttribute('type', 'free')
        joint[i].setAttribute('damping', '0')
        joint[i].setAttribute('armature', '0')
        joint[i].setAttribute('pos', '0 0 0')
        drones[i].appendChild(joint[i])

        site_m0.append(root.createElement('site'))
        site_m0[i].setAttribute('name', 'motor0'+str(i))
        site_m0[i].setAttribute('type', 'cylinder')
        site_m0[i].setAttribute('pos', '0.1 0.1 0.01')
        site_m0[i].setAttribute('size', '0.01 0.0025')
        site_m0[i].setAttribute('quat', '1.0 0.0 0.0 0.0')
        site_m0[i].setAttribute('rgba', '1.0 1.0 1.0 0.5')
        drones[i].appendChild(site_m0[i])

        site_m1.append(root.createElement('site'))
        site_m1[i].setAttribute('name', 'motor1'+str(i))
        site_m1[i].setAttribute('type', 'cylinder')
        site_m1[i].setAttribute('pos', '0.1 -0.1 0.01')
        site_m1[i].setAttribute('size', '0.01 0.0025')
        site_m1[i].setAttribute('quat', '1.0 0.0 0.0 0.0')
        site_m1[i].setAttribute('rgba', '1.0 1.0 1.0 0.5')
        drones[i].appendChild(site_m1[i])

        site_m2.append(root.createElement('site'))
        site_m2[i].setAttribute('name', 'motor2'+str(i))
        site_m2[i].setAttribute('type', 'cylinder')
        site_m2[i].setAttribute('pos', '-0.1 -0.1 0.01')
        site_m2[i].setAttribute('size', '0.01 0.0025')
        site_m2[i].setAttribute('quat', '1.0 0.0 0.0 0.0')
        site_m2[i].setAttribute('rgba', '1.0 1.0 1.0 0.5')
        drones[i].appendChild(site_m2[i])

        site_m3.append(root.createElement('site'))
        site_m3[i].setAttribute('name', 'motor3'+str(i))
        site_m3[i].setAttribute('type', 'cylinder')
        site_m3[i].setAttribute('pos', '-0.1 0.1 0.01')
        site_m3[i].setAttribute('size', '0.01 0.0025')
        site_m3[i].setAttribute('quat', '1.0 0.0 0.0 0.0')
        site_m3[i].setAttribute('rgba', '1.0 1.0 1.0 0.5')
        drones[i].appendChild(site_m3[i])

        geom_thrust0.append(root.createElement('geom'))
        geom_thrust0[i].setAttribute('name', 'thruster0'+str(i))
        geom_thrust0[i].setAttribute('type', 'cylinder')
        geom_thrust0[i].setAttribute('pos', '0.1 0.1 0.01')
        geom_thrust0[i].setAttribute('size', '0.05 0.0025')
        geom_thrust0[i].setAttribute('quat', '1.0 0.0 0.0 0.0')
        geom_thrust0[i].setAttribute('rgba', '1.0 1.0 1.0 0.5')
        geom_thrust0[i].setAttribute('mass', '0.025')
        drones[i].appendChild(geom_thrust0[i])

        geom_thrust1.append(root.createElement('geom'))
        geom_thrust1[i].setAttribute('name', 'thruster1'+str(i))
        geom_thrust1[i].setAttribute('type', 'cylinder')
        geom_thrust1[i].setAttribute('pos', '0.1 -0.1 0.01')
        geom_thrust1[i].setAttribute('size', '0.05 0.0025')
        geom_thrust1[i].setAttribute('quat', '1.0 0.0 0.0 0.0')
        geom_thrust1[i].setAttribute('rgba', '1.0 1.0 1.0 0.5')
        geom_thrust1[i].setAttribute('mass', '0.025')
        drones[i].appendChild(geom_thrust1[i])

        geom_thrust2.append(root.createElement('geom'))
        geom_thrust2[i].setAttribute('name', 'thruster2'+str(i))
        geom_thrust2[i].setAttribute('type', 'cylinder')
        geom_thrust2[i].setAttribute('pos', '-0.1 -0.1 0.01')
        geom_thrust2[i].setAttribute('size', '0.05 0.0025')
        geom_thrust2[i].setAttribute('quat', '1.0 0.0 0.0 0.0')
        geom_thrust2[i].setAttribute('rgba', '1.0 1.0 1.0 0.5')
        geom_thrust2[i].setAttribute('mass', '0.025')
        drones[i].appendChild(geom_thrust2[i])

        geom_thrust3.append(root.createElement('geom'))
        geom_thrust3[i].setAttribute('name', 'thruster3'+str(i))
        geom_thrust3[i].setAttribute('type', 'cylinder')
        geom_thrust3[i].setAttribute('pos', '-0.1 0.1 0.01')
        geom_thrust3[i].setAttribute('size', '0.05 0.0025')
        geom_thrust3[i].setAttribute('quat', '1.0 0.0 0.0 0.0')
        geom_thrust3[i].setAttribute('rgba', '1.0 1.0 1.0 0.5')
        geom_thrust3[i].setAttribute('mass', '0.025')
        drones[i].appendChild(geom_thrust3[i])

        site_qcX.append(root.createElement('site'))
        site_qcX[i].setAttribute('name', 'qcX'+str(i))
        site_qcX[i].setAttribute('pos', '0.1 0 0')
        site_qcX[i].setAttribute('quat', '-0.707107 0 0.707107 0')
        site_qcX[i].setAttribute('size', '0.005 0.1')
        site_qcX[i].setAttribute('type', 'capsule')
        site_qcX[i].setAttribute('rgba', '1 0 0 1')
        drones[i].appendChild(site_qcX[i])

        site_qcY.append(root.createElement('site'))
        site_qcY[i].setAttribute('name', 'qcY'+str(i))
        site_qcY[i].setAttribute('pos', '0 0.1 0')
        site_qcY[i].setAttribute('quat', '0 0 0.707107 0.707107')
        site_qcY[i].setAttribute('size', '0.005 0.1')
        site_qcY[i].setAttribute('type', 'capsule')
        site_qcY[i].setAttribute('rgba', '0 1 0 1')
        drones[i].appendChild(site_qcY[i])

        site_qcZ.append(root.createElement('site'))
        site_qcZ[i].setAttribute('name', 'qcZ'+str(i))
        site_qcZ[i].setAttribute('pos', '0 0 0.1')
        site_qcZ[i].setAttribute('quat', '0.707107 0 0 0')
        site_qcZ[i].setAttribute('size', '0.005 0.1')
        site_qcZ[i].setAttribute('type', 'capsule')
        site_qcZ[i].setAttribute('rgba', '0 0 1 1')
        drones[i].appendChild(site_qcZ[i])
        
        worldbody.appendChild(drones[i])

    #light2 = root.createElement('light')
    #light2.setAttribute('mode', 'targetbodycom')
    #light2.setAttribute('target', 'quadrotor+str(i))
    #light2.setattribute('castshadow', 'false')
    #light1.setAttribute('directional', 'false')
    #light2.setAttribute('diffuse', '0.8 0.8 0.8')
    #light2.setAttribute('specular', '0.3 0.3 0.3')
    #light2.setattribute('pos', '0.0 0.0 4.0')
    #light2.setattribute('dir', '0.0 0.0 -1.0')

    xml.appendChild(compiler)
    xml.appendChild(option)
    xml.appendChild(asset)
    xml.appendChild(worldbody)

    for i in range(N):
        actuator.append(root.createElement('actuator'))
        act_m0.append(root.createElement('motor'))
        act_m0[i].setAttribute('ctrllimited', 'true')
        act_m0[i].setAttribute('ctrlrange', '0.0 2.0')
        act_m0[i].setAttribute('gear', '0.0 0.0 1.0 0.0 0.0 0.0')
        act_m0[i].setAttribute('site', 'motor0'+str(i))
        act_m1.append(root.createElement('motor'))
        act_m1[i].setAttribute('ctrllimited', 'true')
        act_m1[i].setAttribute('ctrlrange', '0.0 2.0')
        act_m1[i].setAttribute('gear', '0.0 0.0 1.0 0.0 0.0 0.0')
        act_m1[i].setAttribute('site', 'motor1'+str(i))
        act_m2.append(root.createElement('motor'))
        act_m2[i].setAttribute('ctrllimited', 'true')
        act_m2[i].setAttribute('ctrlrange', '0.0 2.0')
        act_m2[i].setAttribute('gear', '0.0 0.0 1.0 0.0 0.0 0.0')
        act_m2[i].setAttribute('site', 'motor2'+str(i))
        act_m3.append(root.createElement('motor'))
        act_m3[i].setAttribute('ctrllimited', 'true')
        act_m3[i].setAttribute('ctrlrange', '0.0 2.0')
        act_m3[i].setAttribute('gear', '0.0 0.0 1.0 0.0 0.0 0.0')
        act_m3[i].setAttribute('site', 'motor3'+str(i))
        actuator[i].appendChild(act_m0[i])
        actuator[i].appendChild(act_m1[i])
        actuator[i].appendChild(act_m2[i])
        actuator[i].appendChild(act_m3[i])
        xml.appendChild(actuator[i])
  
    size = root.createElement('size')
    size.setAttribute('njmax', str(120*N))
    size.setAttribute('nconmax', str(40*N))
    xml.appendChild(size)
  
    xml_str = root.toprettyxml(indent ="\t") 
  
    try:
        with open(filename, "w") as f:
            f.write(xml_str) 
        print("\n🎉🎉🎉 Successfully Generated File 🎉🎉🎉\n")
    except:
        print("Failed to write to file")