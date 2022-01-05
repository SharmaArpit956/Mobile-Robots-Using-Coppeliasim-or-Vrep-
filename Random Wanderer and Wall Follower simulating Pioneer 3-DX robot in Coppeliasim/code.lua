function sysCall_init()
    -- initialising 16 inbuilt ultrasonic sensors of the robot by getting their handle
    usensors = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
    for i = 1, 16, 1 do
        usensors[i] = sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor" .. i)
    end

    -- getting handle for  convolute joints of the left and right wheels
    motorLeft = sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight = sim.getObjectHandle("Pioneer_p3dx_rightMotor")

    -- distance set by us, beyond which ultrasonic sensors return something,but we dont want it
    noDetectionDist = 0.5
    -- distance set by us, below which ultrasonic sensors return value,but we dont want that much accuracy
    maxDetectionDist = 0.2

    --an array of detected values
    detect = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

    -- threshold values before which robot should stop or turn to avoid collision with the obstacle(wall)
    frontDistanceThreshold = 0.5
    sidesDistanceThreshold = 0.5

    velocity = 2
    turningVelocity = velocity / 10

    -- velocity for making little turns to adjust itself along the wall at a constant distance
    correctionVelocity = velocity / 20

    --Hysterisis for turnCorrections
    turnCorrectionTolerance = 0.009

    --timer flags
    isDelayRunning = false
    timeSetFlag = 0
    runningTime = 0
    lastTime = 0

    -- time for taking an arc turn
    arcTime = 3
    rotatingTime = 0.7
    -- flag for wandering , 1 means wandering randomly, 0 means not
    wanderingFlag = 1
end

function sysCall_cleanup()
end

function sysCall_actuation()
    read_sensors()

    if noSendorDetected() and wanderingFlag == 1 then
        --  randomly wandering
        wandering()
    else
        -- wanderingFlag = 0 meand stop wandering and start following wall
        if checkObstacleInFrontSide() and detect[9] < frontDistanceThreshold then
            turn(velocity)
        else
            wanderingFlag = 0
            followWall()
        end
    end
end

function wandering()
    -- generate a random number between 1 and 100
    randomNumber = math.random2(1, 100)
    -- please change seed for trying different random patterns
    -- math.randomseed2(24)
    -- --print(randomNumber)
    if randomNumber > 90 then
        --turn left
        turn(velocity)
    elseif randomNumber > 70 then
        --turn right
        turn(-velocity)
    else
        -- move straight
        move(velocity / 2)
    end
end

function followWall()
    read_sensors()
    -- if timer running, dont do anything else
    if isDelayRunning == true then
        delay(runningTime)
    else
        -- read all sensors ad print readings for testing purposes
        -- readSensorsAndPrintValues()
        if wallOver() or uTurn() then
            turnArc(2 * velocity, velocity)
        elseif checkObstacleInFrontSide() then
            turn90(turningVelocity)
        elseif isDelayRunning == false then
            move(velocity)
            -- detect[7]>0 added because wallOver and turnCorrection interfere otherwise
            if detect[7] > 0 then
                -- function for making little turns to adjust itself along the wall at a constant distance
                turnCorrection()
            end
        end
    end
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
-- wall being followed is getting over
function wallOver()
    return (detect[6] == 0 and detect[7] == 0 and detect[8] == 0 and detect[9] > 0 and rearSensorsDetecting() == false)
end

function uTurn()
    -- if only and only one sensor , 9th read positive (due to thickness of the wall),
    -- it means it must complete uTurn
    local sum = 0
    for i = 1, 16, 1 do
        sum = sum + detect[i]
    end
    return (sum == detect[9] and detect[9] > 0)
end

function noSendorDetected()
    local sum = 0
    for i = 1, 16, 1 do
        if detect[i] > frontDistanceThreshold then
            sum = sum + 1
        end
    end
    -- if no sensor detected, then their sum must be 0
    return (sum == 0)
end
-- function for making a turn using differential system as the motors are independently powered
function turn(turnVelocity)
    sim.setJointTargetVelocity(motorLeft, -turnVelocity)
    sim.setJointTargetVelocity(motorRight, turnVelocity)
end

function turn90(turnVelocity)
    sim.setJointTargetVelocity(motorLeft, -turnVelocity)
    sim.setJointTargetVelocity(motorRight, turnVelocity)
    delay(rotatingTime)
end

-- for making an arc , speeds of left and right wheels must be different
function turnArc(leftMotorVelocity, rightMotorVelocity)
    sim.setJointTargetVelocity(motorLeft, leftMotorVelocity)
    sim.setJointTargetVelocity(motorRight, rightMotorVelocity)
    delay(arcTime)
end
-- move straight
function move(velocity)
    sim.setJointTargetVelocity(motorLeft, velocity)
    sim.setJointTargetVelocity(motorRight, velocity)
end

-- checks for the object(wall) in front side and returns true if there is an obstacle on front side
function checkObstacleInFrontSide()
    return (detect[3] > frontDistanceThreshold or detect[4] > frontDistanceThreshold or
        detect[5] > frontDistanceThreshold or
        detect[6] > frontDistanceThreshold)
end

function wallInFront()
    return (detect[4] > frontDistanceThreshold and detect[5] > frontDistanceThreshold)
end
function readSensorsAndPrintValues()
    for i = 1, 16, 1 do
        res, dist = sim.readProximitySensor(usensors[i])
        if (res > 0) and (dist < noDetectionDist) then
            if (dist < maxDetectionDist) then
                dist = maxDetectionDist
            end
            detect[i] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
        else
            detect[i] = 0
        end
        print("Sensor " .. i .. " = " .. detect[i])
    end
end

function delay(time)
    if (timeSetFlag == 0) then
        runningTime = time
        timeSetFlag = 1
        isDelayRunning = true

        lastTime = sim.getSimulationTime()
    -- --print("Timer Started")
    end

    if sim.getSimulationTime() - lastTime > runningTime then
        -- --print("Timer Stopped")
        timeSetFlag = 0
        isDelayRunning = false
    else
    end
end

function turnCorrection()
    -- using hysterisis, if the  diff is between -turnCorrectionTolerance and +turnCorrectionTolerance, do nothing
    -- otherwise, if diff is between below, or beyond  the turnCorrectionTolerance,
    --  make adjustments by making little turns to maintain a constant distance from the wall
    -- if the robot is about to move away from the wall, diff becomes negative, hence it is made to turn right and vice-versa
    local diff = detect[8] - detect[9]

    read_sensors()
     if detect[8] > 0 and detect[9] > 0 then
         if (diff > turnCorrectionTolerance) then
            -- --print("TOO CLOSE/////////")
            --print("Correction-turn left")
            sim.setJointTargetVelocity(motorLeft, -correctionVelocity)
            sim.setJointTargetVelocity(motorRight, correctionVelocity)
        elseif (diff < -turnCorrectionTolerance) then
            -- --print("TOO FAR.........")
            --print("Correction-turn right")
            sim.setJointTargetVelocity(motorLeft, correctionVelocity)
            sim.setJointTargetVelocity(motorRight, -correctionVelocity)
        end
    end
end

function rearSensorsDetecting()
    local s
    s = detect[11] + detect[12] + detect[13] + detect[14] + detect[15]
    return (s > 0)
end
