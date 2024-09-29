#' The code implementation of this assignment was developed collaboratively by 
#' our group and is hosted on GitHub. The repository link is: 
#' https://github.com/mihoyoMhb/AIcourseDemo.
#' Group members:
#' Hangbiao Meng
#' Yuanjing Yang
#' Linjia Zhong

#####################################
#' Quetions1
#' In your function, you set up a function called:
#' 
#' @keywords internal
#' makeRoadMatrices<-function(n){
#'   hroads=matrix(rep(1,n*(n-1)),nrow=n-1)
#'   vroads=matrix(rep(1,(n-1)*n),nrow=n)
#'   list(hroads=hroads,vroads=vroads)
#'}
#'
#'In my opinion, if we have to store the edge cost of the roads, the hroads
#'should be hroads=matrix(rep(1,n*(n-1)),nrow=n) cuz we have n rows? This also
#'be true for vroads=matrix(rep(1,(n-1)*n),nrow=n-1).
#'This question made me feel confused, and it takes me a lot of time to debug.
#'I would be appreciate if you could tell me the difference between these codes!
#####################################

# Main par funtions
# updating and sorting based on the priority
# Our inspiration for writing this function comes from sources:

# Source: Bengü Erenler. Delivery Man. GitHub repository. https://github.com/benguerenler6565/delivery-man
# Source: Rosetta Code. http://rosettacode.org/wiki/Priority_queue#R
PriorityQueue <- function() {
  # Initialize the queues
  # 'queueKeys' means all costs and 'queueValues' means nodes in our A* search
  queueKeys <- numeric()
  queueValues <- list()
  
  
  # Insert method
  insert <- function(key, value) {
    # Initialize index as NULL
    index <- NULL
    
    # Check if the value already exists in queueValues
    if (length(queueValues) > 0) {
      # Find the index of the existing value
      existing_indices <- sapply(queueValues, function(v) identical(v, value))
      index <- which(existing_indices)
    }
    
    if (length(index) > 0) {
      # Value exists, check if the new key is lower (i.e., better)
      index <- index[1]  
      # In case there are multiple, take the first one,
      # 
      if (key < queueKeys[index]) {
        # Remove the old entry with the higher key
        queueKeys <<- queueKeys[-index]
        queueValues <<- queueValues[-index]
      } else {
        # Existing key is better or equal; do not insert
        return()
      }
    }
    
    # Insert the new value and sort the queue
    queueKeys <<- c(queueKeys, key)
    queueValues <<- c(queueValues, list(value))
    # Order the queue based on the keys
    ord <- order(queueKeys)
    queueKeys <<- queueKeys[ord]
    queueValues <<- queueValues[ord]
  }
  #' Source: CRAN. https://search.r-project.org/CRAN/refmans/collections/html/priority_queue.html
  #' Here, the website introduces that there are some methods we need to build
  #' within the queue, they are:
  #' $push(item, priority = 0)
  #' $pop()
  #' $clear().$size()
  #' $as_list()
  #' $print()
  #' But for this assignment, we may just need $pop and an extra $empty method
  pop <- function() {
    if (length(queueValues) == 0) {
      return(NULL)
    }
    head <- queueValues[[1]]
    queueValues <<- queueValues[-1]
    queueKeys <<- queueKeys[-1]
    return(head)
  }
  
  empty <- function() {
    length(queueKeys) == 0
  }
  # Return useful methods so we can call them later
  list(insert = insert, pop = pop, empty = empty)
}

# A simple lists which allows to insert elements on it
# and verity if a particular element exists or not，
# this can be used to check if current node has been used before
Visited_list <- function() {
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
#' This function is to calculate the g(x) in f(x) = g(x) + h(x) 
#' where h(x) means the ManhattanDistance
get_Gx=function(roads, path){
  # initialize cost
  cost = 0
  for(i in 1:(length(path)-1)){
    if(path[[i]][1] == path[[i+1]][1]){ # This means the car will move vertically
      if(path[[i]][2] < path[[i+1]][2])
      {
        cost = cost + roads$vroads[path[[i]][1], path[[i]][2]]
      }else{
        cost = cost + roads$vroads[path[[i+1]][1], path[[i+1]][2]]
      }
    }else{ # or the car will move horizontally
      if(path[[i]][1] > path[[i+1]][1])
      {
        cost = cost + roads$hroads[path[[i+1]][1], path[[i+1]][2]]
      }else{
        cost = cost + roads$hroads[path[[i]][1], path[[i]][2]]
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


A_Search = function(from, to, roads, packages) {
  visited = Visited_list()  # Recording visited nodes
  frontier = PriorityQueue()  # storing nodes that require priority search 
  path = list()  # Recording path
  pathCost = list()  # Recording the cost of paths
  
  frontier$insert(0, from)  # Initializing cost as 0, and inserting the start node 
  pathCost[[paste(from, collapse = ",")]] = 0  # Initializing the cost of path as 0
  while (!frontier$empty()) {
    node = frontier$pop()
    
    # If get the goal node, generating path and return result
    if (node[1] == to[1] && node[2] == to[2]) {
      vectors = list(c(node)) # initialize the path from the node
      curr = paste(node, collapse = ",") # Choose a start
      
      # Recurse the path until we reach the start of path
      while(!all(curr == paste(from, collapse = ","))){
        node = path[[curr]] # Get the previous node
        vectors = c(vectors, list(node)) # Add to the path
        curr = paste(node, collapse = ",")
      }
      # return path from start to our goal
      # print(rev(vectors))
      return (rev(vectors))
      #return (Path_Record(from, node, path))
    }
    
    #' In A* algorithm, we need to check all neighbors of the current
    #' node we are looking at, so we need to define a function to search for
    #' every neighbors
    # Return all available neighbors given a location
    neighbors = matrix(, nrow = 4, ncol=2, byrow = TRUE)
    # Add all possible horizontal and vertical neighbors
    neighbors[,1] = c(node[1] - 1, node[1], node[1], node[1] + 1)
    neighbors[,2] = c(node[2], node[2] + 1, node[2] -1, node[2])
    # Check any illegal neighbors
    neighbors = neighbors[neighbors[,1] > 0,]
    neighbors = neighbors[neighbors[,2] > 0,]
    neighbors = neighbors[neighbors[,1] < dim(roads$hroads)[2]+1,]
    neighbors = neighbors[neighbors[,2] < dim(roads$vroads)[1]+1,]
    for (i in 1:dim(neighbors)[1]) {
      neighbor = neighbors[i,]
      
      # Checking if had visited the node
      if (!visited$exists(neighbor)) {
        # Caculating the cost of path to neighbor node
        currentCost = pathCost[[paste(node, collapse = ",")]] + get_Gx(roads, list(node, neighbor))
        
        # If the neighbor node is already in pathCost and has a higher cost, update it.
        neighborStr = paste(neighbor, collapse = ",")
        if (is.null(pathCost[[neighborStr]]) || currentCost < pathCost[[neighborStr]]) {
          pathCost[[neighborStr]] = currentCost  # Updating the least cost 
          path[[neighborStr]] = node  # updating the frontier of neighbor
          combinedCost = currentCost + get_Hx(neighbor, to) 
          
          # Inserting neighbor's node to priority Queue
          frontier$insert(combinedCost, neighbor)
        }
      }
    }
    visited$insert(node)  # signing the node as visited
  }
}

# Solve the DeliveryMan assignment using the A* search
myFunction=function(roads, car, packages) {
  from = c(car$x, car$y)
  to = NULL
  if(car$load != 0) {
    # If the car is already loaded, then it should go to the delivery location
    # The fifth column == 1 means the package was the chosen one
    to = packages[which(packages[,5] %in% c(1) == TRUE),][3:4]
  } else {
    # Or, to pick up the package
    costs = NULL
    unpicked_package = subset(packages, packages[,5] == 0)
    
    # If there is no package
    if (nrow(unpicked_package) == 0) {
      print("No unpicked packages available.")
      return (NULL)  
    }
    for (i in 1:nrow(unpicked_package)){
      package = unpicked_package[i,]
      pickup_location = package[1:2]
      delivery_location = package[3:4]
      pickup_cost = get_Hx(from, pickup_location)
      #' Question2: Do we need to calculate the pickup_cost and delivery_cost like
      #' package_cost = get_Hx(from, delivery_location)?
      #' or respectively?
      #' package_cost = get_Hx(from, delivery_location)
      #' costs = c(costs, package_cost)
      #' The 'testDM' will raise: 
      #' "You failed to complete the task. Try again."
      #' Additionally, if we ignore the delivery cost, then we will run faster
      #' costs = c(costs, pickup_cost+0*delivery_cost)
      #' testDM(myFunction,verbose=0,returnVec=FALSE,n=500,seed=21,timeLimit=250)
      #' Mean: 171.94
      #' Std Dev: 37.63234
      #' Source: 'Assignment 1: Delivery Man' introduction
      #' or if we set the value like the code below, then we have:
      #' costs = c(costs, pickup_cost+1*delivery_cost)
      #' 178.08 steps
      delivery_cost = get_Hx(pickup_location, delivery_location)
      costs = c(costs, pickup_cost+0*delivery_cost)
    }
    to = unpicked_package[which.min(costs),][1:2]
  }
  
  path = A_Search(from, to, roads, packages)
  nextMove = 0
  # Updating the next states
  if(length(path) == 1) {
    # This happens when the package pickup and delivery locations are equal
    # Then our car stays still
    nextMove <- 5
  }else{
    currX = path[[1]][1]
    currY = path[[1]][2]
    nextX = path[[2]][1]
    nextY = path[[2]][2]
    if (nextX > currX) {
      nextMove <- 6  # Right
    } else if (nextX < currX) {
      nextMove <- 4  # Left
    } else if (nextY > currY) {
      nextMove <- 8  # Up
    } else if (nextY < currY) {
      nextMove <- 2  # Down
    }}
  car$nextMove = nextMove
  return (car)
}