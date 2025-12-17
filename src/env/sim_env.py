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
import math
import random

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

    # Coordinate conventions (important):
    # - USD prim Translate values are expressed in *stage units*.
    # - Stages are commonly authored in centimeters (metersPerUnit=0.01).
    #
    # This helper accepts GO2_DETECTION_CUBE_POS as meters by default, and converts
    # to stage units. If you want to paste values from the Isaac Sim UI "Translate"
    # fields (stage units), set GO2_DETECTION_CUBE_POS_UNITS=stage.
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    pos_units_mode = os.environ.get("GO2_DETECTION_CUBE_POS_UNITS", "meters").strip().lower()
    if pos_units_mode in ("stage", "stage_units", "usd", "units"):
        pos_scale = 1.0
    else:
        pos_scale = 1.0 / meters_per_unit

    for env_idx in range(num_envs):
        # Where to interpret GO2_DETECTION_CUBE_POS:
        # - default: env-local coordinates (good for deterministic placement per env)
        # - optional: world coordinates (useful when env root has unexpected transforms)
        pos_frame = os.environ.get("GO2_DETECTION_CUBE_POS_FRAME", "env").strip().lower()

        env_root = f"/World/envs/env_{env_idx}"
        env_prim = stage.GetPrimAtPath(env_root)
        if not env_prim:
            continue
        pos_units = Gf.Vec3d(*env_position_m) * pos_scale

        if pos_frame == "world":
            env_xform = UsdGeom.Xformable(env_prim)
            env_world_tf = env_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            try:
                pos_units = env_world_tf.GetInverse().Transform(pos_units)
            except Exception:
                # If inverse fails, fall back to env-local.
                pass

        cube_path = f"/World/envs/env_{env_idx}/go2_detection_cube"
        if stage.GetPrimAtPath(cube_path):
            stage.RemovePrim(cube_path)
        cube = UsdGeom.Cube.Define(stage, cube_path)
        cube.CreateSizeAttr(size * (1.0 / meters_per_unit))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOps()
        xform.AddTranslateOp().Set(pos_units)
        cube.GetDisplayOpacityAttr().Set([1.0])

        if os.environ.get("GO2_DEBUG_DETECTION_CUBE", "0").lower() in ("1", "true", "yes"):
            print(
                "[go2_sim] detection cube env_%d pos=%s (%s, frame=%s) -> translate(stage units)=%s metersPerUnit=%.4f"
                % (
                    env_idx,
                    env_position_m,
                    pos_units_mode,
                    pos_frame,
                    (pos_units[0], pos_units[1], pos_units[2]),
                    meters_per_unit,
                )
            )

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


def _world_translation_from_xform(xform: Gf.Matrix4d) -> Gf.Vec3d:
    # Gf.Matrix4d is row-major; translation lives in the last row (m[3][0:3]).
    return Gf.Vec3d(xform[3][0], xform[3][1], xform[3][2])


def _aabb_overlap_3d(a_min: Gf.Vec3d, a_max: Gf.Vec3d, b_min: Gf.Vec3d, b_max: Gf.Vec3d) -> bool:
    return not (
        a_max[0] < b_min[0]
        or a_min[0] > b_max[0]
        or a_max[1] < b_min[1]
        or a_min[1] > b_max[1]
        or a_max[2] < b_min[2]
        or a_min[2] > b_max[2]
    )


def _cube_world_aabb(env_world_tf: Gf.Matrix4d, pos_env_units: Gf.Vec3d, half_units: float) -> tuple[Gf.Vec3d, Gf.Vec3d]:
    # Build 8 corners in env-local and transform to world.
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                p = Gf.Vec3d(
                    pos_env_units[0] + sx * half_units,
                    pos_env_units[1] + sy * half_units,
                    pos_env_units[2] + sz * half_units,
                )
                corners.append(env_world_tf.Transform(p))
    mn = Gf.Vec3d(min(c[0] for c in corners), min(c[1] for c in corners), min(c[2] for c in corners))
    mx = Gf.Vec3d(max(c[0] for c in corners), max(c[1] for c in corners), max(c[2] for c in corners))
    return mn, mx


def spawn_detection_cube_random_near_go2(
    num_envs: int,
    radius_m: float = 20.0,
    min_dist_m: float = 2.0,
    size: float = 0.55,
    color=(0.0, 1.0, 0.0),
    max_tries: int = 80,
):
    """Spawn the detection cube at a random, non-colliding point near the Go2.

    - Samples a point within `radius_m` of the robot (in the env-local frame).
    - Keeps at least `min_dist_m` from the robot.
    - Uses a simple AABB overlap test against *small* boundable prims in the env to
      avoid obvious collisions (skips huge environment meshes/ground).

    Environment variables (optional):
    - GO2_DETECTION_CUBE_POS_UNITS: 'meters' (default) or 'stage'
    - GO2_DETECTION_CUBE_POS_FRAME: 'env' (default) or 'world'
    - GO2_DETECTION_CUBE_RANDOM_RADIUS_M, GO2_DETECTION_CUBE_RANDOM_MIN_DIST_M
    - GO2_DETECTION_CUBE_SIZE, GO2_DETECTION_CUBE_COLOR, GO2_DEBUG_DETECTION_CUBE
    """
    try:
        radius_m = float(os.environ.get("GO2_DETECTION_CUBE_RANDOM_RADIUS_M", radius_m))
    except Exception:
        pass
    try:
        min_dist_m = float(os.environ.get("GO2_DETECTION_CUBE_RANDOM_MIN_DIST_M", min_dist_m))
    except Exception:
        pass
    color = _parse_vec3("GO2_DETECTION_CUBE_COLOR", color)
    try:
        size = float(os.environ.get("GO2_DETECTION_CUBE_SIZE", size))
    except Exception:
        pass

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return

    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    pos_units_mode = os.environ.get("GO2_DETECTION_CUBE_POS_UNITS", "meters").strip().lower()
    if pos_units_mode in ("stage", "stage_units", "usd", "units"):
        pos_scale = 1.0
        radius_units = radius_m
        min_dist_units = min_dist_m
        cube_size_units = size
    else:
        pos_scale = 1.0 / meters_per_unit
        radius_units = radius_m * pos_scale
        min_dist_units = min_dist_m * pos_scale
        cube_size_units = size * pos_scale

    half_units = 0.5 * cube_size_units
    pos_frame = os.environ.get("GO2_DETECTION_CUBE_POS_FRAME", "env").strip().lower()
    debug = os.environ.get("GO2_DEBUG_DETECTION_CUBE", "0").lower() in ("1", "true", "yes")

    # Precompute bounding boxes for collision checks (per env).
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide],
        useExtentsHint=True,
    )

    for env_idx in range(num_envs):
        env_root = f"/World/envs/env_{env_idx}"
        env_prim = stage.GetPrimAtPath(env_root)
        if not env_prim:
            continue

        env_xform = UsdGeom.Xformable(env_prim)
        env_world_tf = env_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        try:
            env_world_inv = env_world_tf.GetInverse()
        except Exception:
            env_world_inv = None

        # Find a robot reference prim (prefer base).
        robot_prim = (
            stage.GetPrimAtPath(f"{env_root}/Go2/base")
            or stage.GetPrimAtPath(f"{env_root}/Go2")
        )
        if not robot_prim:
            continue
        
        # Get robot position in env-local coordinates
        robot_xform = UsdGeom.Xformable(robot_prim)
        robot_world_tf = robot_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        robot_world_pos = _world_translation_from_xform(robot_world_tf)
        
        # Transform robot world position to env-local coordinates
        if env_world_inv is not None:
            robot_env_pos = env_world_inv.Transform(robot_world_pos)
        else:
            # Fallback: try to get the local transform directly
            try:
                # Get the transform relative to the env prim
                robot_local_tf = UsdGeom.Xformable(robot_prim).GetLocalTransformation()
                robot_env_pos = _world_translation_from_xform(robot_local_tf)
            except:
                robot_env_pos = robot_world_pos

        # Collect candidate colliders (small boundables) once.
        colliders = []
        cube_path = f"{env_root}/go2_detection_cube"
        robot_root = f"{env_root}/Go2"
        for prim in Usd.PrimRange(env_prim):
            if not prim.IsActive():
                continue
            p = prim.GetPath().pathString
            if p == cube_path or p.startswith(robot_root):
                continue
            if not prim.IsA(UsdGeom.Boundable):
                continue
            try:
                world_bound = bbox_cache.ComputeWorldBound(prim)
                box = world_bound.ComputeAlignedBox()
                mn = box.GetMin()
                mx = box.GetMax()
                diag = (mx - mn).GetLength()
                # Skip huge bounds (environment meshes/ground); keep obstacles/props.
                if diag > float(max(radius_m * 3.0, 15.0)) * pos_scale:
                    continue
                colliders.append((mn, mx))
            except Exception:
                continue

        # Try to sample a valid location IN WORLD SPACE.
        # This is important because robot_env_pos might be at (0,0,0) in env-local coordinates!
        chosen_world_pos = None
        for _ in range(max_tries):
            # Sample random angle and distance
            theta = random.random() * 2.0 * math.pi
            r_m = min_dist_m + random.random() * max(0.0, radius_m - min_dist_m)
            
            # Calculate candidate position in WORLD coordinates (in meters)
            cand_world_m = Gf.Vec3d(
                robot_world_pos[0] + r_m * math.cos(theta) / meters_per_unit,  # Convert to stage units
                robot_world_pos[1] + r_m * math.sin(theta) / meters_per_unit,
                robot_world_pos[2],  # will set Z below
            )
            
            # Set Z height (in stage units)
            z_default_m = 0.35  # meters above ground
            try:
                z_override_m = float(os.environ.get("GO2_DETECTION_CUBE_Z", z_default_m))
            except Exception:
                z_override_m = z_default_m
            cand_world_m = Gf.Vec3d(cand_world_m[0], cand_world_m[1], z_override_m / meters_per_unit)
            
            # Check minimum distance in XY (in stage units)
            dx = cand_world_m[0] - robot_world_pos[0]
            dy = cand_world_m[1] - robot_world_pos[1]
            dist_sq = (dx * dx + dy * dy) * (meters_per_unit * meters_per_unit)  # Convert to meters squared
            if dist_sq < (min_dist_m * min_dist_m):
                continue
            
            # Check collision with other objects
            # For collision check, we need the AABB in world space
            # But _cube_world_aabb expects env-local pos, so convert temporarily
            if env_world_inv is not None:
                cand_env_temp = env_world_inv.Transform(cand_world_m)
            else:
                cand_env_temp = cand_world_m
            
            cand_world_min, cand_world_max = _cube_world_aabb(env_world_tf, cand_env_temp, half_units)
            if any(_aabb_overlap_3d(cand_world_min, cand_world_max, mn, mx) for mn, mx in colliders):
                continue
            
            chosen_world_pos = cand_world_m
            break
        
        # Convert chosen world position to env-local coordinates for spawning
        if chosen_world_pos is not None:
            if env_world_inv is not None:
                chosen_env_units = env_world_inv.Transform(chosen_world_pos)
            else:
                chosen_env_units = chosen_world_pos
        else:
            chosen_env_units = None

        # Spawn/update cube.
        if stage.GetPrimAtPath(cube_path):
            stage.RemovePrim(cube_path)
        cube = UsdGeom.Cube.Define(stage, cube_path)
        cube.CreateSizeAttr(size * (1.0 / meters_per_unit))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        xform = UsdGeom.Xformable(cube.GetPrim())

        if chosen_env_units is None:
            # Fall back: place in front of robot in world space, then convert to env-local
            fallback_world = Gf.Vec3d(
                robot_world_pos[0] + min_dist_m / meters_per_unit,
                robot_world_pos[1],
                0.35 / meters_per_unit  # meters above ground
            )
            if env_world_inv is not None:
                chosen_env_units = env_world_inv.Transform(fallback_world)
            else:
                chosen_env_units = fallback_world

        xform.AddTranslateOp().Set(chosen_env_units)
        cube.GetDisplayOpacityAttr().Set([1.0])

        # Bind a vivid material (emissive-ish) so color shows up clearly.
        material_path = f"{env_root}/go2_detection_cube_mat"
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


        if debug:
            # Calculate distances for debugging
            dx_m = (chosen_env_units[0] - robot_env_pos[0]) * meters_per_unit
            dy_m = (chosen_env_units[1] - robot_env_pos[1]) * meters_per_unit
            dist_m = math.sqrt(dx_m * dx_m + dy_m * dy_m)
            
            print(
                "[go2_sim] DEBUG SPAWNING env_%d:\n"
                "  robot_world_pos (stage units) = (%.3f, %.3f, %.3f)\n"
                "  robot_env_pos (stage units)   = (%.3f, %.3f, %.3f)\n"
                "  chosen_env_units (stage units) = (%.3f, %.3f, %.3f)\n"
                "  distance from robot = %.3f meters\n"
                "  radius_m=%.1f, min_dist_m=%.1f\n"
                "  meters_per_unit=%.4f"
                % (
                    env_idx,
                    robot_world_pos[0], robot_world_pos[1], robot_world_pos[2],
                    robot_env_pos[0], robot_env_pos[1], robot_env_pos[2],
                    chosen_env_units[0], chosen_env_units[1], chosen_env_units[2],
                    dist_m,
                    radius_m, min_dist_m,
                    meters_per_unit,
                )
            )

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
