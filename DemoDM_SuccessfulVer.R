#' dumbDM
#'
#' This control function just moves randomly, until all packages are picked up and delivered by accident!
#' @param roads See help documentation for the runDeliveryMan function
#' @param cars See help documentation for the runDeliveryMan function
#' @param packages See help documentation for the runDeliveryMan function
#' @return See help documentation for the runDeliveryMan function
#' @export
dumbDM=function(roads,car,packages){
  car$nextMove=sample(c(2,4,6,8),1)
  return (car)
}
#' basicDM
#'
#' This control function will pick up and deliver the packages in the order they
#' are given (FIFO). The packages are then delivered ignoring the trafic conditions
#' by first moving horizontally and then vertically.
#' 
#' As a first step, you should make sure you do better than this.
#' @param roads See help documentation for the runDeliveryMan function
#' @param cars See help documentation for the runDeliveryMan function
#' @param packages See help documentation for the runDeliveryMan function
#' @return See help documentation for the runDeliveryMan function
#' @export
basicDM=function(roads,car,packages) {
  nextMove=0
  toGo=0
  offset=0
  if (car$load==0) {
    toGo=which(packages[,5]==0)[1]
  } else {
    toGo=car$load
    offset=2
  }
  if (car$x<packages[toGo,1+offset]) {nextMove=6}
  else if (car$x>packages[toGo,1+offset]) {nextMove=4}
  else if (car$y<packages[toGo,2+offset]) {nextMove=8}
  else if (car$y>packages[toGo,2+offset]) {nextMove=2}
  else {nextMove=5}
  car$nextMove=nextMove
  car$mem=list()
  return (car)
}
#' manualDM
#'
#' If you have the urge to play the game manually (giving moves 2, 4, 5, 6, or 8 using the keyboard) you
#' can pass this control function to runDeliveryMan
#' @param roads See help documentation for the runDeliveryMan function
#' @param cars See help documentation for the runDeliveryMan function
#' @param packages See help documentation for the runDeliveryMan function
#' @return See help documentation for the runDeliveryMan function
#' @export
manualDM=function(roads,car,packages) {
  if (car$load>0) {
    print(paste("Current load:",car$load))
    print(paste("Destination: X",packages[car$load,3],"Y",packages[car$load,4]))
  }
  car$nextMove=readline("Enter next move. Valid moves are 2,4,6,8,0 (directions as on keypad) or q for quit.")
  if (car$nextMove=="q") {stop("Game terminated on user request.")}
  return (car)
}
averageTest <- function(tests){
  sum = 0
  for (i in 1:tests) {
    sum=sum+runDeliveryMan(A_SearchDM, dim = 10, turns = 2000, doPlot = F, pause = 0, del = 5)
    if(i%%10==0){
      print(i)
      print(sum/i)
    }
  }
  print(sum/i)
  return(0)
}

# Source: http://rosettacode.org/wiki/Priority_queue#R
# Source: https://search.r-project.org/CRAN/refmans/collections/html/priority_queue.html
# AAAAA, 必须修改的代码段落，你们看看吧
PriorityQueue <- function() {
  queueKeys <<- queueValues <<- NULL
  insert <- function(key, value) {
    # 检查是否已经存在该值，并根据代价决定是否替换
    index = getValueIndex(value)
    if(length(index) > 0) {
      if(isTRUE(key < queueKeys[[index]])) {
        queueKeys <<- queueKeys[-index]
        queueValues <<- queueValues[-index]
      } else {
        # 跳过插入操作
        return
      }
    }
    
    # 插入新的值并排序
    temp <- c(queueKeys, key)
    ord <- order(temp)
    queueKeys <<- temp[ord]
    queueValues <<- c(queueValues, list(value))[ord]
  }
  
  pop <- function() {
    head <- queueValues[[1]]
    queueValues <<- queueValues[-1]
    queueKeys <<- queueKeys[-1]
    return (head)
  }
  
  empty <- function() {length(queueKeys) == 0}
  getValueIndex <- function(value) which(queueValues %in% list(value) == TRUE)
  
  list(insert = insert, pop = pop, empty = empty)
}


# A simple lists which allows to insert elements on it
# and verity if a particular element exists or not，
# this can be used to check if current node has been used before
# 本函数也需要被修改
List <- function() {
  listValues <- NULL
  insert <- function(value) {
    valueStr <- paste(value, collapse = ",")
    listValues <<- c(listValues, valueStr)
  }
  exists <- function(value) {
    valueStr <- paste(value, collapse = ",")
    return (valueStr %in% listValues)
  }
  getAllValues <- function() listValues
  list(insert = insert, exists = exists, getAllValues = getAllValues)
}

#' Now it is time to write our own code for A* search algorithm
#' The cost of V_roads and H_roads are different, needs to be calculated separately
#' This function is to calculate the g(x) in f(x) = g(x) + h(x) where h(x) means the ManhattanDistance
get_Gx=function(roads, path){
  # initialize cost
  cost = 0
  for(i in 1:(length(path)-1)){
    MovingVertically = path[[i]][1] == path[[i+1]][1]
    if(MovingVertically){
      if(path[[i]][2] < path[[i+1]][2])
      {
        cost = cost + roads$vroads[path[[i]][2], path[[i]][1]]
      }else{
        cost = cost + roads$vroads[path[[i+1]][2], path[[i+1]][1]]
      }
    }else{
      if(path[[i]][1] > path[[i+1]][1])
      {
        cost = cost + roads$hroads[path[[i+1]][2], path[[i+1]][1]]
      }else{
        cost = cost + roads$hroads[path[[i]][2], path[[i]][1]]
      }
    }
  }
  return (cost)
}

# Manhattan distance for h(x)
get_Hx = function(start_location, end_location){
  return (abs(start_location[1] - end_location[1]) + 
            abs(start_location[2] - end_location[2]))
}

# Get the total cost f(x) = h(x) + g(x)
get_Fx = function(roads, path, temp_goal){
  curr_location = path[[length(path)]][1:2]
  return (get_Gx(roads, path) + get_Hx(curr_location, temp_goal))
}

#' In A* algorithm, we need to check all neighbors of the current
#' node we are looking at, so we need to define a function to search for
#' every neighbors

# Return all available neighbors given a location
Neighbors_search = function(x, y, roads){
  x_limit = dim(roads$hroads)[1]
  y_limit = dim(roads$vroads)[2]
  neighbors = matrix(, nrow = 4, ncol=2, byrow = TRUE)
  # Add all possible horizontal and vertical neighbors
  neighbors[,1] = c(x - 1, x, x, x + 1)
  neighbors[,2] = c(y, y + 1, y -1, y)
  # Check any illegal neighbors
  neighbors = neighbors[neighbors[,1] > 0,]
  neighbors = neighbors[neighbors[,2] > 0,]
  neighbors = neighbors[neighbors[,1] < x_limit+1,]
  neighbors = neighbors[neighbors[,2] < y_limit+1,]
  return (neighbors)
}

Path_Record = function(start_location, end_location, path){
  vectors = list(c(end_location)) # initialize the path from the end_location
  curr = paste(end_location, collapse = ",") # Choose a start
  
  # Recurse the path until we reach the start of path
  while(!all(curr == paste(start_location, collapse = ","))){
    node = path[[curr]] # Get the previous node
    vectors = c(vectors, list(node)) # Add to the path
    curr = paste(node, collapse = ",")
  }
  # return path from start to our goal
  return (rev(vectors))
}
# aStarSearch
A_Search = function(from, to, roads, packages) {
  xSize = dim(roads$hroads)[1]
  ySize = dim(roads$vroads)[2]
  
  visited = List()  # 用于记录已经访问过的节点
  frontier = PriorityQueue()  # 优先队列用于扩展节点
  path = list()  # 用于记录路径
  pathCost = list()  # 新增：用于记录每个节点的路径代价
  
  frontier$insert(0, from)  # 初始代价为0，插入起点
  pathCost[[paste(from, collapse = ",")]] = 0  # 起点路径代价设为0
  while (!frontier$empty()) {
    node = frontier$pop()
    
    # 如果到达目标节点，生成路径并返回
    if (node[1] == to[1] && node[2] == to[2]) {
      return (Path_Record(from, node, path))
    }
    
    neighbors = Neighbors_search(node[1], node[2], roads)
    for (i in 1:dim(neighbors)[1]) {
      neighbor = neighbors[i,]
      
      # 检查是否已经访问过该节点
      if (!visited$exists(neighbor)) {
        # 计算到该邻居节点的路径代价
        currentCost = pathCost[[paste(node, collapse = ",")]] + get_Gx(roads, list(node, neighbor))
        
        # 如果邻居节点已经在 pathCost 中且代价更高，则更新
        neighborStr = paste(neighbor, collapse = ",")
        if (is.null(pathCost[[neighborStr]]) || currentCost < pathCost[[neighborStr]]) {
          pathCost[[neighborStr]] = currentCost  # 更新邻居节点的最小代价
          path[[neighborStr]] = node  # 更新邻居节点的前驱节点
          combinedCost = currentCost + get_Hx(neighbor, to)  # 结合代价计算
          
          # 将邻居节点插入到优先队列中
          frontier$insert(combinedCost, neighbor)
        }
      }
    }
    visited$insert(node)  # 将节点标记为已访问
  }
}

# Given a path, return the best next move car can make towards goal
generateNextMove=function(path) {
  if(isTRUE(length(path) == 1)) {
    # This happens when the package pickup and delivery locations are equal
    return (5)
  }
  
  currX = path[[1]][1]
  currY = path[[1]][2]
  nextX = path[[2]][1]
  nextY = path[[2]][2]
  
  # Move is horizontal
  if (isTRUE(nextX > currX)) {
    return (6) # Right
  }
  if (isTRUE(nextX < currX)) {
    return (4) # Left
  }
  
  # Move is vertical
  if (isTRUE(nextY > currY)) {
    return (8) # Up
  }
  if (isTRUE(nextY < currY)) {
    return (2) # Down
  }
  
  print('Error! Unable to find a suitable move.')
}

# Return a package pickup location which will be used as the goal for a particular search
getPackage=function(from, packages){
  costs = NULL
  unpicked_package = subset(packages, packages[,5] == 0)
  
  # 如果没有包裹需要取，返回空
  if (nrow(unpicked_package) == 0) {
    print("No unpicked packages available.")
    return (NULL)  # 没有包裹
  }
  
  # 计算每个包裹的取送代价
  for (i in 1:nrow(unpicked_package)){
    package = unpicked_package[i,]
    pickup_location = package[1:2]
    delivery_location = package[3:4]
    pickup_cost = get_Hx(from, pickup_location)
    delivery_cost = get_Hx(pickup_location, delivery_location)
    costs = c(costs, pickup_cost + delivery_cost)
  }
  
  # 返回代价最小的包裹
  return (unpicked_package[which.min(costs),])
}

# Solve the DeliveryMan assignment using the A* search
A_SearchDM=function(roads, car, packages) {
  from = c(car$x, car$y)
  to = NULL
  if(car$load != 0) {
    # If the car is already loaded, then it should go to the delivery location
    to = packages[which(packages[,5] %in% c(1) == TRUE),][3:4]
  } else {
    # Or, to pick up the package
    to = getPackage(from, packages)[1:2]
  }
  
  path = A_Search(from, to, roads, packages)
  car$nextMove = generateNextMove(path)
  return (car)
}
#' testDM
#'
#' Use this to debug under multiple circumstances and to see how your function compares with the par function
#' The mean for the par function (with n=500) on this is 172.734, and the sd is approximately 39.065.
#'
#' Your final result will be based on how your function performs on a similar run of 500 games, though with
#' a different seed used to select them.
#'
#' This set of seeds is chosen so as to include a tricky game that has pick ups and deliveries on the same
#' spot. This will occur in the actual games you are evaluated on too.
#'
#'
#' While this is dependent on the machine used, we expect your function to be able to run the 500 evaluation games on
#' the evaluation machine in under 4 minutes (250 seconds). If the evaluation machine is slower than expected,
#' this will be altered so that the required time is 25% slower than the par function.
#'
#' The par function takes approximately 96 seconds on my laptop (with n=500 and verbose=0).
#'
#' @param myFunction The function you have created to control the Delivery Man game.
#' @param verbose Set to 0 for no output, 1 for a summary of the results of the games played (mean,
#' standard deviation and time taken), and 2 for the above plus written output detailing seeds used and the
#' runDeliveryMan output of the result of each game.
#' @param returnVec Set to TRUE if you want the results of the games played returned as a vector.
#' @param n The number of games played. You will be evaluated on a set of 500 games, which is also the default here.
#' @param timeLimit The time limit. If this is breached, a NA is returned.
#' @return If returnVec is false, a scalar giving the mean of the results of the games played. If returnVec is TRUE
#' a vector giving the result of each game played. If the time limit is breached, a NA is returned.
#' @export
testDM=function(myFunction,verbose=0,returnVec=FALSE,n=500,seed=21,timeLimit=250){
  if (!is.na(seed))
    set.seed(seed)
  seeds=sample(1:25000,n)
  startTime=Sys.time()
  aStar=sapply(seeds,function(s){
    midTime=Sys.time()
    if (as.numeric(midTime)-as.numeric(startTime)>timeLimit) {
      cat("\nRun terminated due to slowness.")
      return (NA)
    }
    set.seed(s)
    if (verbose==2)
      cat("\nNew game, seed",s)
    runDeliveryMan(myFunction,doPlot=F,pause=0,verbose=verbose==2)
  })
  endTime=Sys.time()
  if (verbose>=1){
    cat("\nMean:",mean(aStar))
    cat("\nStd Dev:",sd(aStar))
    cat("\nTime taken:",as.numeric(endTime)-as.numeric(startTime),"seconds.")
  }
  if (returnVec)
    return(aStar)
  else
    return (mean(aStar))
}

#' Run Delivery Man
#'
#' Runs the delivery man game. In this game, deliveries are randomly placed on a city grid. You
#' must pick up and deliver the deliveries as fast as possible under changing traffic conditions.
#' Your score is the time it takes for you to complete this task. To play manually pass manualDM
#' as the carReady function and enter the number pad direction numbers to make moves.
#' @param carReady Your function that takes three arguments: (1) a list of two matrices giving the
#' traffice conditions. The first matrix is named 'hroads' and gives a matrix of traffice conditions
#' on the horizontal roads. The second matrix is named 'vroads' and gives a matrix of traffic
#' conditional on the vertical roads. <1,1> is the bottom left, and <dim,dim> is the top right.
#'(2) a list providing information about your car. This
#' list includes the x and y coordinates of the car with names 'x' and 'y', the package the car
#' is carrying, with name 'load' (this is 0 if no package is being carried), a list called
#' 'mem' that you can use to store information you want to remember from turn to turn, and
#' a field called nextMove where you will write what you want the car to do. Moves are
#' specified as on the number-pad (2 down, 4 left, 6 right, 8 up, 5 stay still). (3) A
#' matrix containing information about the packages. This contains five columns and a row for each
#' package. The first two columns give x and y coordinates about where the package should be picked
#' up from. The next two columns give x and y coordinates about where the package should be
#' delivered to. The final column specifies the package status (0 is not picked up, 1 is picked up but not
#' delivered, 2 is delivered).
#' Your function should return the car object with the nextMove specified.
#' @param dim The dimension of the board. You will be scored on a board of dimension 10. Note that
#' this means you will have to remove duplicated nodes from your frontier to keep your AStar
#' computationally reasonable! There is a time limit for how long an average game can be run in, and
#' if your program takes too long, you will penalized or even fail.
#' @param turns The number of turns the game should go for if deliveries are not made. Ignore this
#' except for noting that the default is 2000 so if you have not made deliveries after 2000 turns
#' you fail.
#' @param doPlot Specifies if you want the game state to be plotted each turn.
#' @param pause The pause period between moves. Ignore this.
#' @param del The number of deliveries. You will be scored on a board with 5 deliveries.
#' @return A string describing the outcome of the game.
#' @export
runDeliveryMan <- function (carReady=manualDM,dim=10,turns=2000,
                            doPlot=T,pause=0.1,del=5,verbose=T) {
  roads=makeRoadMatrices(dim)
  car=list(x=1,y=1,wait=0,load=0,nextMove=NA,mem=list())
  packages=matrix(sample(1:dim,replace=T,5*del),ncol=5)
  packages[,5]=rep(0,del)
  for (i in 1:turns) {
    roads=updateRoads(roads$hroads,roads$vroads)
    if (doPlot) {
      makeDotGrid(dim,i)
      plotRoads(roads$hroads,roads$vroads)
      points(car$x,car$y,pch=16,col="blue",cex=3)
      plotPackages(packages)
    }
    if (car$wait==0) {
      if (car$load==0) {
        on=packageOn(car$x,car$y,packages)
        if (on!=0) {
          packages[on,5]=1
          car$load=on
        }
      } else if (packages[car$load,3]==car$x && packages[car$load,4]==car$y) {
        packages[car$load,5]=2
        car$load=0
        if (sum(packages[,5])==2*nrow(packages)) {
          if (verbose)
            cat("\nCongratulations! You suceeded in",i,"turns!")
          return (i)
        }
      }
      car=carReady(roads,car,packages)
      car=processNextMove(car,roads,dim)
    } else {
      car$wait=car$wait-1
    }
    if (pause>0) Sys.sleep(pause)
  }
  cat("\nYou failed to complete the task. Try again.")
  return (NA)
}
#' @keywords internal
packageOn<-function(x,y,packages){
  notpickedup=which(packages[,5]==0)
  onX=which(packages[,1]==x)
  onY=which(packages[,2]==y)
  available=intersect(notpickedup,intersect(onX,onY))
  if (length(available)!=0) {
    return (available[1])
  }
  return (0)
}
#' @keywords internal
processNextMove<-function(car,roads,dim) {
  nextMove=car$nextMove
  if (nextMove==8) {
    if (car$y!=dim) {
      car$wait=roads$vroads[car$y,car$x]
      car$y=car$y+1
    } else {
      warning(paste("Cannot move up from y-position",car$y))
    }
  } else if (nextMove==2) {
    if (car$y!=1) {
      car$y=car$y-1
      car$wait=roads$vroads[car$y,car$x]
    } else {
      warning(paste("Cannot move down from y-position",car$y))
    }
  }  else if (nextMove==4) {
    if (car$x!=1) {
      car$x=car$x-1
      car$wait=roads$hroads[car$y,car$x]
    } else {
      warning(paste("Cannot move left from x-position",car$x))
    }
  }  else if (nextMove==6) {
    if (car$x!=dim) {
      car$wait=roads$hroads[car$y,car$x]
      car$x=car$x+1
    } else {
      warning(paste("Cannot move right from x-position",car$x))
    }
  } else if (nextMove!=5) {
    warning("Invalid move. No move made. Use 5 for deliberate no move.")
  }
  car$nextMove=NA
  return (car)
}

#' @keywords internal
plotPackages=function(packages) {
  notpickedup=which(packages[,5]==0)
  notdelivered=which(packages[,5]!=2)
  points(packages[notpickedup,1],packages[notpickedup,2],col="green",pch=18,cex=3)
  points(packages[notdelivered,3],packages[notdelivered,4],col="red",pch=18,cex=3)
}

#' @keywords internal
makeDotGrid<-function(n,i) {
  plot(rep(seq(1,n),each=n),rep(seq(1,n),n),xlab="X",ylab="Y",main=paste("Delivery Man. Turn ", i,".",sep=""))
}

#' @keywords internal
makeRoadMatrices<-function(n){
  hroads=matrix(rep(1,n*(n-1)),nrow=n)
  vroads=matrix(rep(1,(n-1)*n),nrow=n-1)
  list(hroads=hroads,vroads=vroads)
}

#' @keywords internal
plotRoads<- function (hroads,vroads) {
  for (row in 1:nrow(hroads)) {
    for (col in 1:ncol(hroads)) {
      lines(c(row,row+1),c(col,col),col=hroads[row,col])
    }
  }
  for (row in 1:nrow(vroads)) {
    for (col in 1:ncol(vroads)) {
      lines(c(row,row),c(col,col+1),col=vroads[row,col])
    }
  }
}
#' @keywords internal
updateRoads<-function(hroads,vroads) {
  r1=runif(length(hroads))
  r2=runif(length(hroads))
  for (i in 1:length(hroads)) {
    h=hroads[i]
    if (h==1) {
      if (r1[i]<.05) {
        hroads[i]=2
      }
    }
    else {
      if (r1[i]<.05) {
        hroads[i]=h-1
      } else if (r1[i]<.1) {
        hroads[i]=h+1
      }
    }
    v=vroads[i]
    if (v==1) {
      if (r2[i]<.05) {
        vroads[i]=2
      }
    }
    else {
      if (r2[i]<.05) {
        vroads[i]=v-1
      } else if (r2[i]<.1) {
        vroads[i]=v+1
      }
    }
  }
  list (hroads=hroads,vroads=vroads)
}
