#include "Perception.h"
#include "Utils.h"

#include <queue>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/////////////////////////////////////
/// CONSTRUTOR e FUNCOES PUBLICAS ///
/////////////////////////////////////

Perception::Perception()
{
    started_ = false;
    directionOfNavigation_ = 0.0;
    validDirection_=false;
}

bool Perception::hasValidDirection()
{
    return validDirection_;
}

bool Perception::hasValidMap()
{
    return started_;
}

double Perception::getDirectionOfNavigation()
{
    return directionOfNavigation_;
}

geometry_msgs::msg::PoseStamped& Perception::getDirectionOfNavigationMsg()
{
    return msg_directionOfNavigation_;
}

nav_msgs::msg::OccupancyGrid& Perception::getOccTypesMapMsg()
{
    return msg_occTypes_;
}

nav_msgs::msg::OccupancyGrid& Perception::getPlanTypesMapMsg()
{
    return msg_planTypes_;
}

Pose2D Perception::getLatestPoseFromOdometry()
{
    Pose2D robotPose;

    // Update robotPose from robot transformation
    robotPose.x = odomROS_.pose.pose.position.x;
    robotPose.y = odomROS_.pose.pose.position.y;

    // Convert quaternion to euler angles
    tf2::Quaternion q4(odomROS_.pose.pose.orientation.x,
                       odomROS_.pose.pose.orientation.y, 
                       odomROS_.pose.pose.orientation.z, 
                       odomROS_.pose.pose.orientation.w);
    tf2::Matrix3x3 m4(q4);
    double roll, pitch, yaw;
    m4.getRPY(roll,pitch,yaw);

    // Update orientation with yaw
    robotPose.theta = RAD2DEG(yaw);

    return robotPose;
}

/////////////////////////////////////////////
/// Callbacks dos topicos de MAPA e ODOM  ///
/////////////////////////////////////////////

void Perception::receiveGridmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &value)
{
    // STRUCTURE OF nav_msgs::msg::OccupancyGrid
    // # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    // Header header 
    // 
    // # MetaData for the map
    //   # The time at which the map was loaded
    //   time map_load_time
    //   # The map resolution [m/cell]
    //   float32 resolution
    //   # Map width [cells]
    //   uint32 width
    //   # Map height [cells]
    //   uint32 height
    //   # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
    //   geometry_msgs/Pose origin
    // MapMetaData info
    //
    // # The map data, in row-major order, starting with (0,0).  
    // # Occupancy probabilities are in the range [0,100].  Unknown is -1.
    // # OBS: implemented in c++ with std::vector<u_int8>
    // int8[] data

    if(started_==false){

        // At the first time, initialize all variables and maps
        numCellsX_ = value->info.width;
        numCellsY_ = value->info.height;

        float cellSize = value->info.resolution;

        mapWidth_ = numCellsX_*cellSize;
        mapHeight_ = numCellsY_*cellSize;
        scale_ = 1.0/cellSize;

        occupancyTypeGrid_.resize(numCellsX_*numCellsY_,OCC_UNEXPLORED);

        planningTypeGrid_.resize(numCellsX_*numCellsY_,PLAN_INVALID);
        fValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        gValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        hValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        parentGrid_.resize(numCellsX_*numCellsY_,-1);

        minKnownX_ = numCellsX_-1;
        minKnownY_ = numCellsY_-1;
        maxKnownX_ = maxKnownY_ = 0;

        started_=true;
    }

    // Copy the occupancy grid map to the occupancyTypeGrid_ (which will be modified next)
    for(unsigned int i=0; i<numCellsX_*numCellsY_; i++)
        occupancyTypeGrid_[i] = value->data[i];

    // Classify cells
    updateGridKnownLimits();
    updateCellsClassification();

    // Get cell in free space closest to the robot position
    // This is required because the robot may be near obstacles, 
    // in regions where the planning is not performed
    Pose2D robotPose = getLatestPoseFromOdometry();
    int robotIndexInFreeSpace = getNearestFreeCell(robotPose);

    // Select center of nearest viable frontier
    int nearestFrontierIndex = clusterFrontiersAndReturnIndexOfClosestOne(robotIndexInFreeSpace);
    if(nearestFrontierIndex != -1){

        // Compute A*
        // first - compute heuristic in all cells (euclidian distance to the goal)
        computeHeuristic(nearestFrontierIndex);
        // second - compute the A* algorithm
        int goal = computeShortestPathToFrontier(robotIndexInFreeSpace);

        // Printing the index of the goal cell, must be the same as 'nearestFrontierIndex'
        std::cout << "goal " << goal << std::endl; //

        // Mark path cells for vizualization
        markPathCells(goal);

        // Compute direction of navigation based on the path
        double yaw = computeDirectionOfNavigation(robotIndexInFreeSpace, goal);
        directionOfNavigation_ = normalizeAngleDEG(RAD2DEG(yaw)-robotPose.theta);
        validDirection_=true;

        // Update and publish direction of navigation
        msg_directionOfNavigation_.header = value->header;
        msg_directionOfNavigation_.pose.position.x = robotPose.x;
        msg_directionOfNavigation_.pose.position.y = robotPose.y;
        msg_directionOfNavigation_.pose.position.z = 0;
        tf2::Quaternion quat_tf;
        quat_tf.setRPY( 0, 0, yaw );
        msg_directionOfNavigation_.pose.orientation=tf2::toMsg(quat_tf);
    }else{
        validDirection_=false;
    }

    // Update messages
    msg_occTypes_.header = value->header;
    msg_occTypes_.info = value->info;
    msg_occTypes_.data = occupancyTypeGrid_;

    msg_planTypes_.header = value->header;
    msg_planTypes_.info = value->info;
    msg_planTypes_.data = planningTypeGrid_;

    started_=true;
}

void Perception::receiveOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr &value)
{
    //  STRUCTURE OF nav_msgs::msg::Odometry

    // This represents an estimate of a position and velocity in free space.  
    // The pose in this message should be specified in the coordinate frame given by header.frame_id.
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id
    
    // Header header
    //     # Standard metadata for higher-level stamped data types.
    //     # This is generally used to communicate timestamped data
    //     # in a particular coordinate frame.
    //     #
    //     # sequence ID: consecutively increasing ID
    //     uint32 seq
    //     #Two-integer timestamp that is expressed as:
    //     # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //     # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //     # time-handling sugar is provided by the client library
    //     time stamp
    //     #Frame this data is associated with
    //     # 0: no frame
    //     # 1: global frame
    //     string frame_id
    //              # timestamp in the header is the acquisition time of
    //              # the first ray in the scan.
    //              #
    //              # in frame frame_id, angles are measured around
    //              # the positive Z axis (counterclockwise, if Z is up)
    //              # with zero angle being forward along the x axis
    odomROS_.header = value->header;
    
    odomROS_.child_frame_id = value->child_frame_id;
    
    odomROS_.pose = value->pose;

    odomROS_.twist = value->twist;
}


//////////////////////////////////////////////////////////////////
/// FUNCOES DE PLANEJAMENTO DE CAMINHOS - A SEREM PREENCHIDAS  ///
//////////////////////////////////////////////////////////////////

void Perception::updateCellsClassification()
{
    /// TODO:
    /// varra as celulas dentro dos limites conhecidos do mapa 'occupancyTypeGrid_'
    /// e atualize os valores, marcando como condição de contorno se for o caso

    /// coordenada x das celulas vai de minKnownX_ a maxKnownX_
    /// coordenada y das celulas vai de minKnownY_ a maxKnownY_
    /// compute o indice da celula no grid usando:
    ///    int i = x + y*numCellsX_;
    /// ex: atualizando uma celula como ocupada
    ///    occupancyTypeGrid_[i] = OCC_OCCUPIED;

    /// grid na entrada (valor inteiro): 
    /// - celulas desconhecidas = -1 
    /// - celulas conhecidas com ocupação variando de 0 a 100

    /// grid na saida (valor inteiro):
    /// - celulas desconhecidas                = OCC_UNEXPLORED   (definido como -1)
    /// - celulas de obstaculo                 = OCC_OCCUPIED     (definido como 100)
    /// - celulas livres vizinhas a obstaculos = OCC_NEAROBSTACLE (definido como 90)
    /// - celulas de fronteira                 = OCC_FRONTIER     (definido como 30)
    /// - demais celulas livres                = OCC_FREE         (definido como 50)

    // Dica
    // 1) Marcar obstaculos
    // 2) Marcar celulas vizinhas a obstaculos considerando 'dangerZoneWidth' celulas
    int dangerZoneWidth = 5;
    // 3) Marcar fronteiras (ignorando OCC_OCCUPIED e OCC_NEAROBSTACLE)
    // 4) Marcar restantes, que nao sao inexploradas, como livre

    auto saved_map = occupancyTypeGrid_; // Save the original map
    const auto num_cells_x = numCellsX_;
    const auto num_cells_y = numCellsY_;

    const auto is_near_obstacle = [&saved_map, num_cells_x, num_cells_y](const u_int x, const u_int y, const int occ_type, const int zoneWidth) -> bool {
        for (int dx = -zoneWidth; dx <= zoneWidth; ++dx) {
            for (int dy = -zoneWidth; dy <= zoneWidth; ++dy) {
                if (dx == 0 && dy == 0) continue; // Skip the cell itself
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < static_cast<int>(num_cells_x) && ny >= 0 && ny < static_cast<int>(num_cells_y)) {
                    int neighborIndex = nx + ny * num_cells_x;
                    if (saved_map[neighborIndex] == occ_type) {
                        return true;
                    }
                }
            }
        }
        return false;
    };

    constexpr int RAW_OCCUPIED_THRESHOLD = 100; // Threshold for occupied cells
    const auto mark_occupied = [&saved_map, num_cells_x](const u_int x, const u_int y) {
        const auto cell = x + y * num_cells_x;
        if (saved_map[cell] >= RAW_OCCUPIED_THRESHOLD)
        {
            saved_map[cell] = OCC_OCCUPIED;
        }
    };

    const auto mark_free = [&saved_map, num_cells_x](const u_int x, const u_int y) {
        const auto cell = x + y * num_cells_x;
        if (saved_map[cell] != OCC_OCCUPIED)
        {
            saved_map[cell] = OCC_FREE;
        }
    };

    const auto mark_near_obstacle = [&saved_map, num_cells_x, dangerZoneWidth, &is_near_obstacle](const u_int x, const u_int y) {
        const auto cell = x + y * num_cells_x;
        if (saved_map[cell] != OCC_OCCUPIED && is_near_obstacle(x, y, OCC_OCCUPIED, dangerZoneWidth))
        {
            saved_map[cell] = OCC_NEAROBSTACLE;
        }
    };

    constexpr auto NEAR_UNEXPLORED = 1; // Width of the zone to consider for unexplored cells
    const auto mark_frontier = [&saved_map, num_cells_x, &is_near_obstacle](const u_int x, const u_int y)
    {
        const auto cell = x + y * num_cells_x;
        if (saved_map[cell] == OCC_FREE && is_near_obstacle(x, y, OCC_UNEXPLORED, NEAR_UNEXPLORED))
        {
            saved_map[cell] = OCC_FRONTIER;
        }
    };

    const auto mark_functions = std::vector<std::function<void(u_int, u_int)>>{
        mark_occupied,
        mark_free,
        mark_near_obstacle,
        mark_frontier,
    };

    for (auto mark : mark_functions)
    {
        for (u_int x = minKnownX_; x <= maxKnownX_; x++)
        {
            for (u_int y = minKnownY_; y <= maxKnownY_; y++)
            {
                int i = x + y * num_cells_x;
                if (saved_map[i] != OCC_UNEXPLORED) // Only process known cells
                {
                    mark(x, y);
                }
            }
        }
    }

    // Copy the updated map back to the occupancyTypeGrid_
    occupancyTypeGrid_ = saved_map;
}

void Perception::computeHeuristic(int goalIndex)
{
    int goalX = goalIndex % numCellsX_;
    int goalY = goalIndex / numCellsX_;

    /// TODO:
    /// varra as celulas dentro dos limites conhecidos do mapa 'planningTypeGrid_'
    /// e atualize os valores das medidas f, g, h e pi das celulas validas
    
    /// coordenada x das celulas vai de minKnownX_ a maxKnownX_
    /// coordenada y das celulas vai de minKnownY_ a maxKnownY_
    /// compute o indice da celula no grid usando:
    ///    int i = x + y*numCellsX_;

    /// Dica: uma celula 'i' eh valida se (planningTypeGrid_[i] != PLAN_INVALID)

    /// A atualizacao deve seguir as seguintes regras:
    ///   fValueGrid_[i] e gValueGrid_[i] - valores f e g: recebem DBL_MAX (equivalente ao infinito)
    ///   parentGrid_[i] - valor pi (indicando o pai da celula): recebe -1, pois a priori nao aponta para ninguem. 
    ///   hValueGrid_[i] - valor h - distancia para o goal

    const auto heuristic = [goalX, goalY](u_int x, u_int y) {
        const double dx = static_cast<double>(x - goalX);
        const double dy = static_cast<double>(y - goalY);
        return sqrt(dx * dx + dy * dy); // Euclidean distance
    };

    for (u_int x = minKnownX_; x <= maxKnownX_; x++)
    {
        for (u_int y = minKnownY_; y <= maxKnownY_; y++)
        {
            int i = x + y * numCellsX_;
            if (planningTypeGrid_[i] != PLAN_INVALID)
            {
                fValueGrid_[i] = DBL_MAX;
                gValueGrid_[i] = DBL_MAX;
                parentGrid_[i] = -1;
                hValueGrid_[i] = heuristic(x, y);
            }
        }
    }
}

// offset para os 8 vizinhos
// uso, i-esimo vizinho (nx,ny) da posicao (x,y):
//      int nx = x+offset[i][0];
//      int ny = y+offset[i][1];
int offset[8][2] = {{-1,  1}, { 0,  1}, { 1,  1}, { 1,  0}, { 1, -1}, { 0, -1}, {-1, -1}, {-1,  0}};

// custo de distancia para os 8 vizinhos
// uso, atualizando custo do i-esimo vizinho
//      int id_celula  =  x +  y*numCellsX_;
//      int id_vizinho = nx + ny*numCellsX_;
//      gValueGrid_[id_vizinho] = gValueGrid_[id_celula] + cost[i];
double cost[8] = {sqrt(2), 1, sqrt(2), 1, sqrt(2), 1, sqrt(2), 1};

int Perception::computeShortestPathToFrontier(int robotCellIndex)
{
    [[maybe_unused]] u_int rx = robotCellIndex % numCellsX_;
    [[maybe_unused]] u_int ry = robotCellIndex / numCellsX_;

    /// TODO:
    /// Computar o algoritmo A Star usando os valores em hValueGrid_ 
    /// e atualizando os valores em fValueGrid_, gValueGrid_ e parentGrid_

    /// Ao fim deve retornar o indice da celula de goal, encontrada na busca. Ou -1 se nao encontrar
    int goal = -1;

    /// Sugestao: usar a fila de prioridades abaixo
    /// onde o primeiro elemento do par eh o f-value e o segundo elemento eh o indice da celula
    std::priority_queue< std::pair<double, int> , std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>> > pq;

    /// Exemplo: insercao na fila
    //      std::pair<double, int> vizinho;
    //      vizinho.first = fValueGrid_[id_vizinho];
    //      vizinho.second = id_vizinho;
    //      pq.push(vizinho);

    /// Exemplo: remocao da fila
    //      std::pair<double, int> celula = pq.top();
    //      pq.pop();
    //      int id_celula = celula.second;

    /// O algoritmo comeca da posicao do robo
    gValueGrid_[robotCellIndex] = 0;

    std::pair<double, int> inicio;
    inicio.first = fValueGrid_[robotCellIndex];
    inicio.second = robotCellIndex;

    pq.push(inicio);

    /// Completar algoritmo A Star, consultando a fila enquanto ela nao estiver vazia
    ///     while(!pq.empty())

    while (!pq.empty())
    {
        const auto top = pq.top();
        pq.pop();

        const auto curr_idx = top.second;

        if (planningTypeGrid_[curr_idx] == PLAN_GOALS)
        {
            // Encontrou o goal
            goal = curr_idx;
            break;
        }

        u_int curr_x = curr_idx % numCellsX_;
        u_int curr_y = curr_idx / numCellsX_;

        for (int k = 0; k < 8; k++)
        {
            u_int nx = curr_x + offset[k][0];
            u_int ny = curr_y + offset[k][1];
            u_int neighbor_idx = nx + ny * numCellsX_;

            // Verifica se o vizinho e valido
            if (nx < minKnownX_ || nx > maxKnownX_ || ny < minKnownY_ || ny > maxKnownY_ || planningTypeGrid_[neighbor_idx] == PLAN_INVALID)
                continue;

            // Calcula o custo g do vizinho
            double tentative_g = gValueGrid_[curr_idx] + cost[k];

            // Se o custo g for melhor, atualiza os valores
            if (tentative_g < gValueGrid_[neighbor_idx])
            {
                gValueGrid_[neighbor_idx] = tentative_g;
                fValueGrid_[neighbor_idx] = tentative_g + hValueGrid_[neighbor_idx];
                parentGrid_[neighbor_idx] = curr_idx;

                // Adiciona o vizinho na fila de prioridades
                std::pair<double, int> vizinho;
                vizinho.first = fValueGrid_[neighbor_idx];
                vizinho.second = neighbor_idx;
                pq.push(vizinho);
            }
        }
    }

    return goal;
}

////////////////////////////////////////////////////////
/// FUNCOES AUXILIARES PARA PLANEJAMENTO DE CAMINHOS ///
////////////////////////////////////////////////////////

// Given the explored area, update the following variables: minKnownX_, maxKnownX_, minKnownY_, maxKnownY_
void Perception::updateGridKnownLimits()
{
    for(int x=0; x<numCellsX_; x++){
        for(int y=0; y<numCellsY_; y++){
            int i = x + y*numCellsX_;
            if(occupancyTypeGrid_[i]!=-1)
            {
                if(x<minKnownX_) minKnownX_ = x;
                if(y<minKnownY_) minKnownY_ = y;
                if(x>maxKnownX_) maxKnownX_ = x;
                if(y>maxKnownY_) maxKnownY_ = y;
            }
        }
    }
}

// Groups all frontier cells in clusters, and keep the ones with size greater than 'minFrontierSize' 
// Then selects the centers of each of the frontiers
// Next, choose the one closest to the robot's position as the goal
// Returns the index of the cell associated with the center of the selected frontier
int Perception::clusterFrontiersAndReturnIndexOfClosestOne(int robotCellIndex)
{
    frontierCentersIndices.clear();

    int width=1;
    int minFrontierSize = 3;

    // Check occupancyTypeGrid_ and set PLAN_GOALS in planningTypeGrid_
    for(int x=minKnownX_; x<=maxKnownX_; x++){
        for(int y=minKnownY_; y<=maxKnownY_; y++){
            int i = x + y*numCellsX_;

            if(occupancyTypeGrid_[i] == OCC_FRONTIER) 
                planningTypeGrid_[i] = PLAN_GOALS;      // Frontier cells are goals
            else if(occupancyTypeGrid_[i] == OCC_FREE) 
                planningTypeGrid_[i] = PLAN_REGULAR;    // Free cells are regular cells (where path can be computed)
            else
                planningTypeGrid_[i] = PLAN_INVALID;    // Remaining cells are invalid for planning
        }
    }

    // Group all neighboring goal cells
    for(int x=minKnownX_; x<=maxKnownX_; x++){
        for(int y=minKnownY_; y<=maxKnownY_; y++){
            int i = x + y*numCellsX_;

            // detect a goal cell that is not MARKED yet
            if(planningTypeGrid_[i] == PLAN_GOALS){
                planningTypeGrid_[i] = PLAN_MARKEDGOALS;

                std::vector<unsigned int> frontier;

                float centerx = 0, centery = 0;
                float count = 0;

                // mark all neighbor goal cells
                // breadth-first search using a queue
                std::queue<int> q;
                q.push(i);
                while(!q.empty())
                {
                    int c = q.front();
                    q.pop();
                    frontier.push_back(c);

                    int cx = c % numCellsX_;
                    int cy = c / numCellsX_;
                    centerx += cx;
                    centery += cy;
                    count++;

                    for(int nx=cx-width;nx<=cx+width;nx++){
                        for(int ny=cy-width;ny<=cy+width;ny++){
                            int ni = nx + ny*numCellsX_;
                            if(planningTypeGrid_[ni] == PLAN_GOALS){
                                planningTypeGrid_[ni] = PLAN_MARKEDGOALS;
                                q.push(ni);
                            }
                        }
                    }
                }

                // keep frontiers that are larger than minFrontierSize
                if(count > minFrontierSize){
                    centerx /= count;
                    centery /= count;
 
                    // find cell closest to frontier center
                    float minDist=FLT_MAX;
                    int closest=-1;

                    for(unsigned int k=0;k<frontier.size();k++){
                        int fx = frontier[k] % numCellsX_;
                        int fy = frontier[k] / numCellsX_;

                        float dist = sqrt(pow(fx-centerx,2.0)+pow(fy-centery,2.0));
                        if(dist < minDist){
                            minDist = dist;
                            closest = frontier[k];
                        }
                    }

                    // add center of frontier to list of Goals
                    frontierCentersIndices.push_back(closest);

                }else{

                    // ignore small frontiers
                    for(unsigned int k=0;k<frontier.size();k++){
                        planningTypeGrid_[frontier[k]] = PLAN_REGULAR;
                    }
                }
            }
        }
    }

    // These are the filtered frontiers (that are not too small)
    std::cout << "Number of frontiers: " << frontierCentersIndices.size() << std::endl;
    for(unsigned int k=0;k<frontierCentersIndices.size();k++){
        planningTypeGrid_[frontierCentersIndices[k]] = PLAN_GOALS;
    }

    if(frontierCentersIndices.empty())
        return -1;
    else{

        // Select nearest frontier among the filtered frontiers
        int nearestFrontierIndex=-1;
        float distance = DBL_MAX;

        int rx = robotCellIndex % numCellsX_;
        int ry = robotCellIndex / numCellsX_;
        for(int k=0;k<frontierCentersIndices.size();k++){
            int nFx = frontierCentersIndices[k] % numCellsX_;
            int nFy = frontierCentersIndices[k] / numCellsX_;
            float d = sqrt(pow(rx-nFx,2.0)+pow(ry-nFy,2.0));
            if(d < distance)
            {
                distance = d;
                nearestFrontierIndex = frontierCentersIndices[k];
            }
        }

        // Clear frontiers that were not selected
        for(int k=0;k<frontierCentersIndices.size();k++){
            if(frontierCentersIndices[k] != nearestFrontierIndex)
                planningTypeGrid_[frontierCentersIndices[k]] = PLAN_MARKEDGOALS;
        }

        return nearestFrontierIndex;
    }
}

// Mark all path cells in the 'planningTypeGrid_' after the A* Star algorithm is computed
// by checking the index of the parent of each cell starting from the goal
void Perception::markPathCells(int goal)
{
    if(goal != -1){

        int c = parentGrid_[goal];

        while(c != -1){
            planningTypeGrid_[c] = PLAN_PATH;
            c = parentGrid_[c];
        }
    }
}

// Select a path cell in a distance given by 'localGoalRadius' from the robot
// and compute the angle difference from the robot orientation to this cell
double Perception::computeDirectionOfNavigation(int robotCellIndex, int goalIndex)
{
    int rx = robotCellIndex % numCellsX_;
    int ry = robotCellIndex / numCellsX_;

    int c = goalIndex;

    double localGoalRadius = 3;

    int cx, cy;

    while(parentGrid_[c] != -1){
        cx = c % numCellsX_;
        cy = c / numCellsX_;

        double dist = sqrt(pow(cx-rx,2.0)+pow(cy-ry,2.0));
        
        if(dist < localGoalRadius){
            break;
        }
        c = parentGrid_[c];
    }

    double yaw = atan2(cy-ry,cx-rx);

    return yaw;
}

// Return index of the free cell closest to the robot position
int Perception::getNearestFreeCell(Pose2D robot)
{
    int rx = robot.x*scale_ + numCellsX_/2;
    int ry = robot.y*scale_ + numCellsY_/2; 

    int u;

    for(int l=1; l<20; l++){

        for(int cx=rx-l; cx<=rx+l; cx++){
            u = cx + (ry+l)*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
            u = cx + (ry-l)*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
        }

        for(int cy=ry-l; cy<=ry+l; cy++){
            u = rx+l + cy*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
            u = rx-l + cy*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
        }

    }

    return -1;
}
