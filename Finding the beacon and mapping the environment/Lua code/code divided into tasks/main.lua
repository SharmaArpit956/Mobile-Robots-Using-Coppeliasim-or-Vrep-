function loadRobotfile()
    local f = assert(loadfile("d:/Workspace/MR Main CourseWork/main/robot.lua"))
    return f()
end

function loadGridMapFile()
    local f = assert(loadfile("d:/Workspace/MR Main CourseWork/main/grid_map.lua"))
    return f()
end
function loadTask1And2File()
    local f = assert(loadfile("d:/Workspace/MR Main CourseWork/main/task_1_and_2.lua"))
    return f()
end
function loadTask3File()
    local f = assert(loadfile("d:/Workspace/MR Main CourseWork/main/task3.lua"))
    return f()
end

function loadTask4File()
    local f = assert(loadfile("d:/Workspace/MR Main CourseWork/main/task4.lua"))
    return f()
end
function sysCall_init()
    loadRobotfile()
    -- loadGridMapFile()

    initRobot()
    loadTask1And2File()
    loadTask3File()
    loadTask4File()
    -- initGridMap()

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
