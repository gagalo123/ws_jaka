from pathlib import Path
import argparse
import sys
import math
import time
import numpy as np
import sys
try:
    import pinocchio as pin
except Exception as e:
    print("需要安装 pinocchio：pip install pinocchio 或使用系统包。错误：", e)
    sys.exit(1)


try:
    # from meshcat.visualizer import Visualizer
    # from meshcat.geometry import Sphere
    # from pinocchio.visualize import MeshcatVisualizer
    from pinocchio.visualize import MeshcatVisualizer
    from meshcat.geometry import Sphere
    MESHCAT_AVAILABLE = True
except Exception:
    MESHCAT_AVAILABLE = False

def build_models_from_urdf(urdf_path: Path, mesh_dir: Path, use_freeflyer=False):
    # 选择 joint model 类型
    joint_model = pin.JointModelFreeFlyer() if use_freeflyer else pin.JointModelRX()
    # pin.buildModelFromUrdf 能接收 Path 对象
    # model = pin.buildModelFromUrdf(str(urdf_path), joint_model if use_freeflyer else None)
    # 加载碰撞几何（COLLISION 或 VISUAL，根据 URDF 中标签）
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
                                   urdf_path,
                                   str(mesh_dir))
    geom_model = pin.buildGeomFromUrdf(model,
                                   urdf_path,
                                   pin.GeometryType.COLLISION,
                                   package_dirs=[str(mesh_dir)])
    return model, geom_model, visual_model, collision_model

def load_srdf_if_exists(geom_model, model, srdf_path: Path):
    if srdf_path and srdf_path.exists():
        try:
            pin.removeCollisionPairs(model, geom_model, str(srdf_path))
            pin.loadReferenceConfigurations(model, str(srdf_path))
            print(f"已应用 SRDF: {srdf_path}")
            return True
        except Exception as e:
            print("加载 SRDF 失败，继续但忽略 SRDF。错误：", e)
    return False
def remove_collision_pair(geom_model,geom_data,start_joint_name:str,end_joint_name:str):
    for k in range(len(geom_model.collisionPairs)-1):
        cp = geom_model.collisionPairs[k]
        first_geom = geom_model.geometryObjects[cp.first].name  
        second_geom = geom_model.geometryObjects[cp.second].name
        if first_geom == start_joint_name and second_geom == end_joint_name:
            geom_model.removeCollisionPair(cp)
            break
    geom_data = pin.GeometryData(geom_model)
    return geom_data
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', type=str, default='/home/mofang/code/ws_jaka/install/dual_arm/share/dual_arm/urdf/dual_arm.urdf', help='URDF 文件路径（必需）')
    parser.add_argument('--srdf', type=str, default=None, help='SRDF 文件路径（可选，用于移除 collision pairs 与参考位姿）')
    parser.add_argument('--mesh-dir', type=str, default='/home/mofang/code/ws_jaka/install/dual_arm/share/dual_arm/meshes', help='mesh 文件目录（可选，默认为 URDF 的目录）')
    parser.add_argument('--steps', type=int, default=100, help='随机目标总次数')
    parser.add_argument('--interp-steps', type=int, default=40, help='插值步数（每次运动的细分步）')
    parser.add_argument('--meshcat', action='store_true', help='开启 Meshcat 粗略可视化（可选）')
    parser.add_argument('--sleep', type=float, default=0.02, help='每个插值步的休眠时间，便于观察（秒）')
    parser.add_argument('--stop-at-first-collision', action='store_true', help='检测到第一个碰撞即停止该运动（默认 False）')
    args, unknown = parser.parse_known_args()
    
    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        print('找不到 URDF 文件：', urdf_path)
        sys.exit(1)
    mesh_dir = Path(args.mesh_dir) if args.mesh_dir else urdf_path.parent
    print('URDF 路径：', urdf_path)
    print('Mesh 目录：', mesh_dir)
    print('加载模型（URDF）...')
    model, geom_model, visual_model, collision_model = build_models_from_urdf(urdf_path, mesh_dir, use_freeflyer=False)

    
    srdf_path = Path(args.srdf) if args.srdf else None
    if srdf_path:
        load_srdf_if_exists(geom_model, model, srdf_path)
        print(f"碰撞对数量：{len(geom_model.collisionPairs)}")
    else:
        # 如果 srdf 不存在，仍然可以通过 geom_model.addAllCollisionPairs() 来添加
        try:
            geom_model.addAllCollisionPairs()
            print("已添加所有碰撞对（无 SRDF）。")
        except Exception:
        # 有些 pinocchio 版本没有 addAllCollisionPairs 方法
            pass
    
    q_cur = np.zeros(model.nq)
    try:
        if srdf_path and srdf_path.exists() and 'referenceConfigurations' in dir(model) and len(model.referenceConfigurations) > 0:
            # 尝试取第一个 reference 配置
            key = list(model.referenceConfigurations.keys())[0]
            q_cur = model.referenceConfigurations[key].copy()
            print('使用 SRDF 中的参考位姿：', key)
    except Exception:
        q_cur = np.zeros(model.nq)

    data = model.createData()
    geom_data = pin.GeometryData(geom_model)
    
    
    
    igno_coll_pairs=[['l6_0','l7_0'],['r6_0','r7_0']]
    for pair in igno_coll_pairs:
        print(f"Ignoring collision pair: {pair}")
        geom_data = remove_collision_pair(geom_model, geom_data,pair[0],pair[1])
    
    
    vis = None
    if args.meshcat and MESHCAT_AVAILABLE:
        viz = MeshcatVisualizer(model, collision_model, visual_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()
        q0 = pin.neutral(model)
        pin.forwardKinematics(model, data, q0)
        pin.updateGeometryPlacements(model, data, geom_model, geom_data)
        viz.display(q0)
        viz.displayVisuals(True)
        nframes = len(model.frames)
    q_tgt = q0
    for t in range(args.steps):
        # 随机生成一个目标位姿
        # q_tgt = np.random.uniform(low=model.lowerPositionLimit, high=model.upperPositionLimit)
        q_last = q_tgt
        q_tgt = pin.randomConfiguration(model)
        print(f"第 {t+1} 次初始坐标：", np.round(q_last, 3))
        print(f"第 {t+1} 次目的目标：", np.round(q_tgt, 3))
        for i in range(args.interp_steps):
            alpha = (i + 1) / args.interp_steps
            q_cur = (1 - alpha) * q_cur + alpha * q_tgt
            pin.forwardKinematics(model, data, q_cur)
            pin.updateGeometryPlacements(model, data, geom_model, geom_data)
            viz.display(q_cur)
            stop_at_first_collision = bool(args.stop_at_first_collision)
            
            pin.computeCollisions(geom_model, geom_data, stop_at_first_collision)
            
            

            
            
            in_collision = False
            for k in range(len(geom_model.collisionPairs)):
                cr = geom_data.collisionResults[k]
                cp = geom_model.collisionPairs[k]
                
                if(cr.isCollision()):
                    collision_detected = True
                    # 获取碰撞对中的几何体名称
                    first_geom = geom_model.geometryObjects[cp.first].name
                    second_geom = geom_model.geometryObjects[cp.second].name
            
                    # self.vis.viewer[first_geom].set_property("color", [1, 0, 0, 1])
                    # self.vis.viewer[second_geom].set_property("color", [1, 0, 0, 1])
                    print(f"\033[91m碰撞发生在: {first_geom} 和 {second_geom}\033[0m")
                    
                    input("按回车继续...")
            print(f"Step {t+1}, Interp {i+1}, q: ", np.round(q_cur, 3))
            if args.sleep > 0:
                    time.sleep(args.sleep)
        time.sleep(2)
    print("测试完成。")
    
    
    input("按回车开始...")
    
        