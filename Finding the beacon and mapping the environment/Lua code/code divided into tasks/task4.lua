function task4Init()
  
    hokuyo = sim.getObjectHandle("fastHokuyo")
    goalP ={0,0,0}

    pose = updateRobotPose()

    wheel_radius = 0.195 / 2
    b = 0.1655
    stop_distance = 0.2
    e = 0.3
    wL = 0
    wR = 0
    motionParams = {stop_distance, e, wheel_radius, b}

    scan_angle = math.rad(sim.getUserParameter(hokuyo, "scanAngle"))
    cell_size = 0.1
    window_size = 2 * sim.getUserParameter(hokuyo, "maxScanDistance")
    sector_angle = math.rad(5)
    safety_dist = 0.55
    wide_sector_width = 5
    tau_low = 500
    tau_high = 2000
    target_weight = 3
    previous_weight = 2
    current_weight = 1
    vfhParams = {
        scan_angle,
        cell_size,
        window_size,
        sector_angle,
        safety_dist,
        wide_sector_width,
        tau_low,
        tau_high,
        target_weight,
        previous_weight,
        current_weight
    }

    sector_target =
        sim.addDrawingObject(
        sim.drawing_lines,
        2,
        0,
        -1,
        math.ceil(2 * math.pi / sector_angle),
        nil,
        nil,
        nil,
        {0, 0, 0}
    )
    sector_masked =
        sim.addDrawingObject(
        sim.drawing_lines,
        2,
        0,
        -1,
        math.ceil(2 * math.pi / sector_angle),
        nil,
        nil,
        nil,
        {0, 1, 0}
    )
    sector_candidates =
        sim.addDrawingObject(
        sim.drawing_lines,
        2,
        0,
        -1,
        math.ceil(2 * math.pi / sector_angle),
        nil,
        nil,
        nil,
        {0, 1, 1}
    )
    sector_selected = sim.addDrawingObject(sim.drawing_lines, 2, 0, -1, 1, nil, nil, nil, {0, 0, 1})

    selected = 0
    target = 0
    masked = {}
    candidates = {}

    max_velocity = 0.4

    simVFHp.init(vfhParams, motionParams)

    laserPoints = {}
    sonarData = {}
end

function task4CleanUp()
    sim.removeDrawingObject(sector_target)
    sim.removeDrawingObject(sector_masked)
    sim.removeDrawingObject(sector_candidates)
    sim.removeDrawingObject(sector_selected)
    simVFHp.release()
end

function task4Sensing()
    laserPoints = getLaserPoints()
end

function task4()
    local completed = false
    local wL, wR
    pose = updateRobotPose()
    primary, binary, masked = simVFHp.getHistogram(laserPoints, sonarData)
    selected, target, candidates = simVFHp.getDirection(masked, pose, goalP)
    DrawSectors()
    wL, wR = simVFHp.getMotion(max_velocity, selected, pose, goalP)
    sim.setJointTargetVelocity(motorLeft, wL)
    sim.setJointTargetVelocity(motorRight, wR)
    return completed
end

function getLaserPoints()
    local laserScan
    local laserPts = {}
    local j = 1
    laserScan = sim.callScriptFunction("getMeasuredData@fastHokuyo", sim.scripttype_childscript)
    for i = 1, #laserScan, 3 do
        laserPts[j] = {laserScan[i], laserScan[i + 1]}
        j = j + 1
    end
    return laserPts
end

function updateRobotPose()
    local pose
    position = sim.getObjectPosition(ref_point_robot, -1)
    orientation = sim.getObjectOrientation(ref_point_robot, -1)
    pose = {position[1], position[2], orientation[3]}
    return pose
end

function DrawSectors()
    local pos, ori
    pos = sim.getObjectPosition(hokuyo, -1)
    ori = sim.getObjectOrientation(hokuyo, -1)
    sim.addDrawingObjectItem(sector_candidates, nil)
    for i = 1, #candidates, 1 do
        DrawSector(sector_candidates, pos, ori, candidates[i], safety_dist, 0.031)
    end
    sim.addDrawingObjectItem(sector_masked, nil)
    for i = 1, #masked, 1 do
        if (not masked[i]) then
            DrawSector(sector_masked, pos, ori, (i - 0.5) * sector_angle, safety_dist / 2, 0.032)
        end
    end
    sim.addDrawingObjectItem(sector_selected, nil)
    DrawSector(sector_selected, pos, ori, selected, safety_dist / 3, 0.034)
    sim.addDrawingObjectItem(sector_target, nil)
    DrawSector(sector_target, pos, ori, target, safety_dist / 4, 0.033)
end

function DrawSector(sector, pos, ori, angle, length, offset)
    local x, y, line
    line = {pos[1], pos[2], pos[3] + offset, 0, 0, pos[3] + offset}
    x = length * math.cos(angle)
    y = length * math.sin(angle)
    line[4] = math.cos(ori[3]) * x - math.sin(ori[3]) * y + pos[1]
    line[5] = math.sin(ori[3]) * x + math.cos(ori[3]) * y + pos[2]
    sim.addDrawingObjectItem(sector, line)
end
