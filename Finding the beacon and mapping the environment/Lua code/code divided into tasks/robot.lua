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

function  turn90(turningVelocity)
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
