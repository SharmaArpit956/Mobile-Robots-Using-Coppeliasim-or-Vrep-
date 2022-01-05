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
    frontDistanceThreshold = 0.4

    velocity = 2

    -- velocity for making little turns to adjust itself along the wall at a constant distance
    correctionVelocity = velocity / 10

    --Hysterisis for turnCorrections
    turnCorrectionTolerance = 0.001
    toleranceGap = turnCorrectionTolerance * 6

    -- flag for wandering , 1 means wandering randomly, 0 means not
    wanderingFlag = 1

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
    -- -- print("error is : " .. setPoint - measuredDistance)

    x = updatePID(setPoint - measuredDistance, measuredDistance)
    -- -- print(x)

    -- the output between -10 and 10 is maped to -velocity/10 to velocity/10, and assigned to correctionVelocity
    correctionVelocity = map(x, -10, 10, -velocity / 10, velocity / 10)
    -- print(correctionVelocity)
end

function sysCall_cleanup()
end

function sysCall_actuation()
    readSensorsAndPrintValues()

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

-- Re-maps a number from one range to another
-- same function as in Arduino IDE
function map(value, fromLow, fromHigh, toLow, toHigh)
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
end

function wandering()
    -- generate a random number between 1 and 100
    randomNumber = math.random2(1, 100)
    -- please change seed for trying different random patterns
    -- math.randomseed2(24)
    -- ---- -- -- print(randomNumber)
    if randomNumber > 90 then
        --turn right
        turn(-velocity)
    elseif randomNumber > 70 then
        --turn left
        turn(velocity)
    else
        -- move straight
        move(velocity / 2)
    end
end

function followWall()
    read_sensors()
    print(detect[9])
    if checkObstacleInFrontSide() then
        print("turning")
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

--USED FOR RANDOM WANDERING ONLY
-- returns true, if none of the 16 sensors is detecting anything
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

--USED FOR RANDOM WANDERING ONLY
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

--USED FOR RANDOM WANDERING ONLY
-- checks for the object(wall) in front side and returns true if there is an obstacle on front side
function checkObstacleInFrontSide()
    return (detect[3] > 0 or detect[4] > 0 or
        detect[5] > frontDistanceThreshold or
        detect[6] > frontDistanceThreshold or
        detect[7] > frontDistanceThreshold)
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
        -- -- -- print("Sensor " .. i .. " = " .. detect[i])
    end
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

-- checks if the robot is aligned with wall or not
function alignedWithWall()
    local diff = math.abs(detect[8] - detect[9])
    if (diff < turnCorrectionTolerance) then
        return true
    else
        return false
    end
end

--checks if the measured distance is equal to the set distanc eor not
function atProperDistance()
    return (detect[8] > frontDistanceThreshold and detect[8] < frontDistanceThreshold + toleranceGap)
end

-- Does the turning angle corrections by varying turning-speed, calculated from the output of the PID controller
function turnCorrectionForPID()
    -- if alignedWithWall() == false or atProperDistance() == false then
    --     read_sensors()
    pid()
    if correctionVelocity > 0 then
        -- print("Left Angle Correction")
    else
        -- print("Right Angle Correction")
    end
    sim.setJointTargetVelocity(motorLeft, -correctionVelocity)
    sim.setJointTargetVelocity(motorRight, correctionVelocity)
    -- end
end
