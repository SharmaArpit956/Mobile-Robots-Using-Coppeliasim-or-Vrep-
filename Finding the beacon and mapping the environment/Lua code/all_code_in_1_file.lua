function sysCall_init()
    initRobot()
    --initialize every task's variables
    task1lInit()
    task2Init()
    task3Init()
    task4Init()

    -- Names for each defined state of the algorithm
    States = {"Measuring Wall", "Finding Middle", "Task 2", "Task 3", "Task 4", "Stop"}
    state = 1

    -- variable for co-ordinates of the middle of the room
    innerRoomMiddle = {}
end

function sysCall_cleanup()
    -- simGridMap.saveMap('map.bmp')
    -- simGridMap.release()
    task4CleanUp()
end

function sysCall_actuation()
    -- gridMapRunning()
    read_sensors()

    if state == 1 then
        index = measuringWall()
        if index > 4 then
            state = 2
        end
    elseif state == 2 then
        local completed = findingMiddle()
        if completed then
            innerRoomMiddle = sim.getObjectPosition(ref_point_robot, -1)
            goalP = innerRoomMiddle
            state = 3
        end
    elseif state == 3 then
        local completed = task2()
        if completed then
            state = 4
        end
    elseif state == 4 then
        local completed = task3()
        local currentPosition = sim.getObjectPosition(ref_point_robot, -1)
        if completed then
            state = 5
        end
    elseif state == 5 then
        print("Task 4 should begin - BACK HOME ")
        local completed = task4()
    end
end

function sysCall_sensing()
    task4Sensing()
    --    gridMapSensing()
end

function initRobot()
    -- initialising 16 inbuilt ultrasonic sensors of the robot by getting their handle
    usensors = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
    for i = 1, 16, 1 do
        usensors[i] = sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor" .. i)
    end

    -- getting handle for  convolute joints of the left and right wheels
    motorLeft = sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight = sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    ref_point_robot = sim.getObjectHandle("Pioneer_p3dx_connection1")
    -- distance set by us, beyond which ultrasonic sensors return something,but we dont want it
    noDetectionDist = 0.5
    -- distance set by us, below which ultrasonic sensors return value,but we dont want that much accuracy
    maxDetectionDist = 0.2

    iPos = 0

    --an array of detected values
    detect = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

    velocity = 2
    turningVelocity = velocity / 4.9

    turningInitFlag = 0
    angleTurned = 0
    previousAngle = 0
end

function moveDistance(velocity, D)
    local pos
    local movingCompleted = false
    if movingInitFlag == 0 then
        movingInitFlag = 1
        iPos = sim.getObjectPosition(ref_point_robot, -1)
    end
    pos = sim.getObjectPosition(ref_point_robot, -1)
    local d = getDistanceBetweenCoordinates(iPos, pos)
    if d > D then
        movingInitFlag = 0
        movingCompleted = true
        stop()
    else
        move(velocity)
    end
    return d, movingCompleted
end

function turn90(turningVelocity)
    local rotAmount = -math.pi / 2
    local sign = rotAmount / math.abs(rotAmount)
    sim.setJointTargetVelocity(motorLeft, turningVelocity * sign)
    sim.setJointTargetVelocity(motorRight, -turningVelocity * sign)

    if turningInitFlag == 0 then
        turningInitFlag = 1
        angleTurned = 0
        previousAngle = sim.getObjectOrientation(ref_point_robot, -1)[3]
    -- print("dx is :" .. angle - previousAngle)
    end

    local angle = sim.getObjectOrientation(ref_point_robot, -1)[3]
    local dx = angle - previousAngle

    if dx >= 0 then
        dx = math.mod(dx + math.pi, 2 * math.pi) - math.pi
    else
        dx = math.mod(dx - math.pi, 2 * math.pi) + math.pi
    end
    angleTurned = angleTurned + dx
    previousAngle = angle
    local turningCompleted = math.abs(angleTurned) > (math.pi - 0.13) / 2
    if turningCompleted then
        turningInitFlag = 0
    end
    return turningCompleted
end

-- function for making a turn using differential system as the motors are independently powered
function turn(turnVelocity)
    sim.setJointTargetVelocity(motorLeft, -turnVelocity)
    sim.setJointTargetVelocity(motorRight, turnVelocity)
end

-- move straight
function move(velocity)
    sim.setJointTargetVelocity(motorLeft, velocity)
    sim.setJointTargetVelocity(motorRight, velocity)
end

-- stop for 1 second
function stop()
    sim.setJointTargetVelocity(motorLeft, 0)
    sim.setJointTargetVelocity(motorRight, 0)
end

function read_sensors()
    -- in the  following, 0 < detect[i] < 1, larger the distance fro object(wall) , smaller the value
    for i = 1, 16, 1 do
        -- sim.readProximitySensor(usensors[i]) returns 2 values , res>0 means something detected
        --dist is the value for calculating the distance
        --dist< noDetectionDist means, the read value is less than the upper limit of the required range
        res, dist = sim.readProximitySensor(usensors[i])
        --if something is detected and the read value is less than the upper limit of the required range
        if (res > 0) and (dist < noDetectionDist) then
            --if  the read value is less than the lower limit of the required range,
            --make it equal to the lower limit
            if (dist < maxDetectionDist) then
                dist = maxDetectionDist
            end
            -- value subtracted from 1 because as distance increases detect[i] decreases and vice-versa
            detect[i] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
        else
            -- no object/obstacle, 0 reading
            detect[i] = 0
        end
    end
end

function wallInFront()
    return (detect[4] > 0 and detect[5] > 0)
end

function task1lInit()
    -- measuring Wall variables Initialization
    measuringWallStates = {"move", "Turning In Degrees", "Stop"}
    index = 1
    measuringWallstate = 1

    -- co-ordinates of the corners of the wall for calculating length and breadth of the room
    wallCornersCoordinates = {}

    -- finding Middle variables Initialization
    findingMiddleStates = {"move Half Length", "Turning In Degrees", "move Half Breadth", "Stop"}
    findingMiddleState = 1
    movingInitFlag = 0

    -- Variables for inner room's length and breadth to be measured by the robot
    innerRoomLength = 0
    innerRoomBreadth = 0
end

function task2Init()
    task2States = {"Move ahead", "Move back", "Turn 90", "Move A distance"}
    task2State = 1
    --distance D, can be seen in algorithm of task 2 in the report
    D = 0
end

-- function for finding the center of the room
function findingMiddle()
    local completed = false
    if findingMiddleState == 1 then
        -- 0.1 is subtracted to correct the error as the reference point is not exactly in the center of the robot
        -- which effects measurements on rotation a bit
        innerRoomLength = getDistanceBetweenCoordinates(wallCornersCoordinates[4], wallCornersCoordinates[3]) - 0.1
        l, _ = moveDistance(-velocity, innerRoomLength / 2)
        if l > innerRoomLength / 2 then
            findingMiddleState = 2
        end
    elseif findingMiddleState == 2 then
        local turningcompleted = turn90(-turningVelocity)
        if turningcompleted then
            findingMiddleState = 3
        end
    elseif findingMiddleState == 3 then
        innerRoomBreadth = getDistanceBetweenCoordinates(wallCornersCoordinates[2], wallCornersCoordinates[3]) - 0.1
        l, _ = moveDistance(velocity, innerRoomBreadth / 2)
        if l > innerRoomBreadth / 2 then
            findingMiddleState = 4
        end
    elseif findingMiddleState == 4 then
        D = innerRoomBreadth / 2
        completed = true
        stop()
    end
    return completed
end

-- function for measuring the length and breadth of the room
function measuringWall()
    if measuringWallstate == 1 then
        move(velocity)
        if wallInFront() then
            if turningInitFlag == 0 then
                wallCornersCoordinates[index] = sim.getObjectPosition(ref_point_robot, -1)
                index = index + 1
            end
            measuringWallstate = 2
        end
    elseif measuringWallstate == 2 then
        local turningcompleted = turn90(-turningVelocity)
        if turningcompleted then
            if index < 5 then
                measuringWallstate = 1
            end
        end
    end
    return index
end
-- distance between 2 co-ordinates (x1,y1) and (x2,y2)= ?((x1-x2)^2+(y1-y2)^2)
function getDistanceBetweenCoordinates(a, b)
    local deltaX = a[1] - b[1]
    local deltaY = a[2] - b[2]
    return math.sqrt(deltaX ^ 2 + deltaY ^ 2)
end

function task2()
    local task2Complete = false
    if task2State == 1 then
        local distanceMoved
        local movingComplete
        -- D*1.1 because we want robot to move a bit more than D to detect is there is a wall in front or not
        distanceMoved, movingComplete = moveDistance(velocity, D * 1.1)
        if movingComplete then
            if wallInFront() then
                task2State = 2
            else
                task2State = 4
            end
        end
    elseif task2State == 2 then
        local distanceMoved
        local movingComplete
        distanceMoved, movingComplete = moveDistance(-velocity, D)
        if movingComplete then
            task2State = 3
        end
    elseif task2State == 3 then
        local turningcompleted = turn90(-turningVelocity)
        if turningcompleted then
            task2State = 1
        end
    elseif task2State == 4 then
        local movingComplete
        -- _ is used as we dont need current moved distance here
        _, movingComplete = moveDistance(velocity, 4)
        if movingComplete then
            task2Complete = true
        end
    end
    return task2Complete
end

function task3Init()
    -- threshold values before which robot should stop or turn to avoid collision with the obstacle(wall,sofa,or ring)
    frontDistanceThreshold = 0.4

    velocity = 2

    -- velocity for making little turns to adjust itself along the wall at a constant distance
    correctionVelocity = velocity / 10

    --PID variables
    dStateFlag = 0
    setPoint = frontDistanceThreshold
    dState = 0 -- Last position input, getRobotPosition
    iState = 0 -- Integrator state
    iMax = 3 -- Maximum allowable integrator state
    iMin = 2 --  Minimum allowable integrator state
    iGain = 53 -- integral gain
    pGain = 397 -- proportional gain
    dGain = 55 -- derivative gain

    TurnToMoveRatio = 5
    ratioCounter = 0

    task3States = {"move Straight", "PID follower", "Locate Beacon", "Face away from beacon"}
    task3State = 1
end

function task3()
    local completed = false
    read_sensors()

    if task3State == 1 then
        move(velocity)
        if wallInFront() then
            task3State = 2
        end
    elseif task3State == 2 then
        if checkSensorsInLeftSide() == true then
            task3State = 3
        elseif checkObstacleInFrontSide() then
            turn(velocity / 10)
        else
            if ratioCounter > TurnToMoveRatio then
                move(velocity)
            else
                turnCorrectionForPID()
            end
            if ratioCounter > 10 then
                ratioCounter = 0
            else
                ratioCounter = ratioCounter + 1
            end
        end
    elseif task3State == 3 then
        print("Left Sensors Triggered")
        if detect[4] > 0.5 and detect[5] > 0.5 then
            task3State = 4
        elseif detect[4] > 0 and detect[5] > 0 then
            move(velocity)
        else
            turn(turningVelocity)
        end
    elseif task3State == 4 then
        stop()
        if detect[4] > 0 or detect[5] > 0 then
            turn(-velocity)
        else
            stop()
            completed = true
        end
    end
    return completed
end

function pid()
    -- P  I  D

    if dStateFlag == 0 then
        dStateFlag = 1
        dState = detect[8]
    end
    if detect[5] == 0 then
        measuredDistance = 1 - detect[8]
    else
        measuredDistance = 1 - detect[5]
    end

    x = updatePID(setPoint - measuredDistance, measuredDistance)

    -- the output between -10 and 10 is maped to -velocity/10 to velocity/10, and assigned to correctionVelocity
    correctionVelocity = map(x, -10, 10, -velocity / 10, velocity / 10)
end

-- Re-maps a number from one range to another
-- same function as in Arduino IDE
function map(value, fromLow, fromHigh, toLow, toHigh)
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
end

function updatePID(error, position)
    local pTerm
    local dTerm
    local iTerm

    --P
    -- calculate the proportional term
    pTerm = pGain * error

    --I
    -- calculate the integral state with appropriate limiting
    iState = iState + error
    if (iState > iMax) then
        iState = iMax
    elseif (iState < iMin) then
        iState = iMin
    end
    -- calculate the integral term
    iTerm = iGain * iState

    --D
    dTerm = dGain * (position - dState)
    dState = position

    return pTerm + iTerm - dTerm
end

-- Does the turning angle corrections by varying turning-speed, calculated from the output of the PID controller
function turnCorrectionForPID()
    pid()
    turn(correctionVelocity)
end

function checkObstacleInFrontSide()
    return (detect[4] > 0 or detect[5] > frontDistanceThreshold or detect[6] > frontDistanceThreshold or
        detect[7] > frontDistanceThreshold)
end

function checkSensorsInLeftSide()
    return (detect[16] > 0 or detect[1] or detect[2] or detect[3])
end

function task4Init()
    hokuyo = sim.getObjectHandle("fastHokuyo")
    goalP = {0, 0, 0}

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
