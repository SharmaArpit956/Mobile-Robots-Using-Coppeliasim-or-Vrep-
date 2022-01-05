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
