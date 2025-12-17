from isaacsim.core.utils.prims import define_prim, get_prim_at_path
try:
    import isaacsim.storage.native as nucleus_utils
except ModuleNotFoundError:
    import isaacsim.core.utils.nucleus as nucleus_utils
from isaaclab.terrains import TerrainImporterCfg, TerrainImporter
from isaaclab.terrains import TerrainGeneratorCfg
from env.terrain_cfg import HfUniformDiscreteObstaclesTerrainCfg
import omni.replicator.core as rep
import omni.usd
from pxr import UsdGeom, Gf, UsdShade, Usd
import os

def add_semantic_label():
    ground_plane = rep.get.prims("/World/GroundPlane")
    with ground_plane:
    # Add a semantic label
        rep.modify.semantics([("class", "floor")])


def _parse_vec3(env_var: str, default):
    raw = os.environ.get(env_var, "").strip()
    if not raw:
        return default
    try:
        parts = [float(p) for p in raw.split(",")]
        if len(parts) == 3:
            return tuple(parts)
    except Exception:
        pass
    return default


def spawn_detection_cube(
    num_envs: int,
    # Position in the env frame (meters). This is intentionally *not* relative to the robot
    # because the Go2 prims are managed by IsaacLab and can be moved by physics/replication.
    # Using a fixed env-frame position makes the test object deterministic and easy to find.
    env_position_m=(3.0, 1.0, 0.35),
    size=0.55,
    color=(0.0, 1.0, 0.0),
):
    """Place a colored cube in each environment for detector sanity checks."""
    env_position_m = _parse_vec3("GO2_DETECTION_CUBE_POS", env_position_m)
    color = _parse_vec3("GO2_DETECTION_CUBE_COLOR", color)
    try:
        size = float(os.environ.get("GO2_DETECTION_CUBE_SIZE", size))
    except Exception:
        pass

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return

    # Isaac stages are commonly authored in centimeters (metersPerUnit=0.01).
    # Our offsets/sizes are expressed in meters, so convert to stage units.
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    unit_scale = 1.0 / meters_per_unit

    for env_idx in range(num_envs):
        # Place in the env frame so it always spawns "somewhere else", not intersecting the robot.
        env_root = f"/World/envs/env_{env_idx}"
        env_prim = stage.GetPrimAtPath(env_root)
        if not env_prim:
            continue
        env_xform = UsdGeom.Xformable(env_prim)
        env_world_tf = env_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos_units = Gf.Vec3d(*env_position_m) * unit_scale
        cube_pos = env_world_tf.Transform(pos_units)

        cube_path = f"/World/envs/env_{env_idx}/go2_detection_cube"
        if stage.GetPrimAtPath(cube_path):
            stage.RemovePrim(cube_path)
        cube = UsdGeom.Cube.Define(stage, cube_path)
        cube.CreateSizeAttr(size * unit_scale)
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOps()
        xform.AddTranslateOp().Set(cube_pos)
        cube.GetDisplayOpacityAttr().Set([1.0])

        # Bind a vivid material (emissive-ish) so color shows up clearly.
        material_path = f"/World/envs/env_{env_idx}/go2_detection_cube_mat"
        shader_path = f"{material_path}/PBRShader"
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, shader_path)
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateDiffuseColorAttr(Gf.Vec3f(*color))
        shader.CreateEmissiveColorAttr(Gf.Vec3f(*color))
        shader.CreateRoughnessAttr(0.2)
        shader.CreateMetallicAttr(0.0)
        material.CreateSurfaceOutput().ConnectToSource(shader, "surface")
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

def create_obstacle_sparse_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=100 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 

def create_obstacle_medium_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=200 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 


def create_obstacle_dense_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=400,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 

def create_warehouse_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_forklifts_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_shelves_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    prim.GetReferences().AddReference(asset_path)

def create_full_warehouse_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_hospital_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Hospital")
    prim = define_prim("/World/Hospital", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Hospital/hospital.usd"
    prim.GetReferences().AddReference(asset_path)

def create_office_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Office")
    prim = define_prim("/World/Office", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Office/office.usd"
    prim.GetReferences().AddReference(asset_path)

def create_grid_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Grid")
    prim = define_prim("/World/Grid", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Grid/default_environment.usd"
    prim.GetReferences().AddReference(asset_path)
