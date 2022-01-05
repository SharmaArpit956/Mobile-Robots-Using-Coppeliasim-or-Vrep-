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
-- distance between 2 co-ordinates (x1,y1) and (x2,y2)= √((x1-x2)^2+(y1-y2)^2)
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
