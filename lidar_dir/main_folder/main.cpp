//#include <math.h>
#include "headers.h"



//#include "matplotlibcpp.h"
using namespace sl;
float* myX; float* myY;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define baliseRadius 0.04


#include <fcntl.h>

int verbose;

lidarPos myPos;
lidarPos filteredPos;
Opponent myOpponent;
bool filteredPosAcquired = false;
FILE* logFile;

beaconAbsolutePos beaconRefPosition[3];
lidarPos calibPos;
pthread_mutex_t filteredPositionLock;

int startPosition = 6;
typedef enum{BLUE, YELLOW} TEAM_COLOR;
TEAM_COLOR myColor;


float perimetre = 8.6; /*8.4 sur l'arène!!!!*/
float limit_of_detection = 3.6;
double objectMaxStep = 0.12; //ETAIT A 0.9 JE LAI CHANGE AJD
double max_object_width = 0.2;
int angleTolerance = 30;
float triangleErrorTolerance = 0.15;//il est à 0.08 par défaut
float isoceleTolerance = 0.15; //ETAIT A 0.08
float longSideLength = 3.326; //Longueur des deux grands côtés du triangle
float shortSideLength = 1.9; //Longueur des deux petits côtés du triangle
pthread_mutex_t positionLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t isReadyLock =PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t lidarDataLock = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t lockOpponentPosition = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t lockMyState = PTHREAD_MUTEX_INITIALIZER;
uint8_t readyToSend = 0;
uint8_t lidarDataCopied = 0;

typedef enum{CALIB_MODE, LOCALIZE_MODE} PROGRAM_STATE;
PROGRAM_STATE myState;


void *safe_malloc(size_t size, int id) {
    printf("before safe_malloc of id  = %d\n",id);
    void *ptr = malloc(size);
    if (ptr == NULL) {
        fprintf(stderr, "Erreur : l'allocation de mémoire a échoué. ID malloc = %d \n",id);
        exit(EXIT_FAILURE);
    }
    printf("After safe_malloc of id  = %d\n",id);
    return ptr;
}





double distance (double a1,double a2,double d1,double d2){
    double dist= sqrt(pow(d1,2)+pow(d2,2) - (2*d1*d2*cos((a1-a2)*M_PI/180)));
    //if(verbose) fprintf(stderr,"distance is %lf",dist);
    return dist;
}

double beaconDistance(beaconAbsolutePos beacon1, beaconAbsolutePos beacon2){
    return pow(pow(beacon1.x - beacon2.x,2)+pow(beacon1.y - beacon2.y,2),0.5);
}

float angleDiff(float a1, float a2) {
    // Calculer la différence entre deux angles en tenant compte de leur nature cyclique
    float diff = a2 - a1;
    if (diff < 0) diff += 360;
    return diff;
}



float triangulationPierlot(float *x, float *y,
						float alpha1, float alpha2, float alpha3,
						float x1, float y1, float x2, float y2, float x3, float y3)
{
	float cot_12 = Cot( alpha2 - alpha1 ) ;
	float cot_23 = Cot( alpha3 - alpha2 ) ;
	cot_12 = adjust_value_to_bounds( cot_12 , COT_MAX ) ;
	cot_23 = adjust_value_to_bounds( cot_23 , COT_MAX ) ;
	float cot_31 = ( 1.0 - cot_12 * cot_23 ) / ( cot_12 + cot_23 ) ;
	cot_31 = adjust_value_to_bounds( cot_31 , COT_MAX ) ;
	
	float x1_ = x1 - x2 , y1_ = y1 - y2 , x3_ = x3 - x2 , y3_ = y3 - y2 ;

	float c12x = x1_ + cot_12 * y1_ ;
	float c12y = y1_ - cot_12 * x1_ ;

	float c23x = x3_ - cot_23 * y3_ ;
	float c23y = y3_ + cot_23 * x3_ ;

	float c31x = (x3_ + x1_) + cot_31 * (y3_ - y1_) ;
	float c31y = (y3_ + y1_) - cot_31 * (x3_ - x1_) ;
	float k31 = (x3_ * x1_) + (y3_ * y1_) + cot_31 * ( (y3_ * x1_) - (x3_ * y1_) ) ;
  
  float D = (c12x - c23x) * (c23y - c31y) - (c23x - c31x) * (c12y - c23y) ;
  float invD = 1.0 / D ;
  float K = k31 * invD ;
    pthread_mutex_lock(&positionLock);
	*x = K * (c12y - c23y) + x2 ;
	*y = K * (c23x - c12x) + y2 ;
    pthread_mutex_unlock(&positionLock);
    //fprintf(stderr,"pos pierlot x = %fand y = %f \n",*x,*y);
	
	return NULL ; /* return 1/D */
}



void* beacon_data(void* argument){
    float distanceToCenter;
    float oldError = INFINITY;
    if(verbose) printf("rentre dans beaconData\n");
    float oldPerimetre = 0;
    float oldIsoceleCondition = INFINITY;
    float debugPerimetre;
    float debugIsoceleCondition;
    float debugBalises[3][2];
    
    /*for(int i = 0; i<counter; i++){
        if(verbose) fprintf(stderr,"Point at angle %f and distance %f \n",a[i],d[i]);

    }*/
    if(verbose) fprintf(stderr,"entre dans beaco_data \n");

    pthread_mutex_lock(&lidarDataLock);
    lidar_data *myData = (lidar_data*) argument;
    float a[myData->counter];
    float d[myData->counter];
    // Copier les valeurs du tableau original dans le tableau copie
    for(int i = 0; i < myData->counter; i++) {
        a[i] = myData->angle[i];
        d[i] = myData->distance[i];
    }

    int counter = myData->counter;
    //pthread_mutex_lock(&printLock);
    //fprintf(stderr,"counter = %d in beacondata \n",counter);
    lidarDataCopied = 1;
    pthread_mutex_unlock(&lidarDataLock);
    for(int i = 0; i <counter; i++) {
        /*if(a[i]>360)*/// printf("angle dans beacon = %f \n",a[i]);
    }
    //pthread_mutex_unlock(&printLock);

    if(verbose) fprintf(stderr,"We detected %d points in beacon_data \n",counter);
    //std::ifstream file;
    //file.open("lidar_2112_v2.txt");
    //std::cout<<"c'est beacons qui marche pas?";
    //printf("counter=%d", counter);
    int obj_counter=0;
    std::vector<float> newa;
    std::vector<float> newd;
    std::vector<float> newWidth;
    //float newa[167]={};
    //float newd[167]={};
    float refd=d[0];
    float refa=a[0];
    float moyd=refd;
    float moya=refa;
    int obj_iter=0;
    int moy_count=1;
    double actualDistance;
    std::vector<lidarPos> objects_coordinates;
    double beaconFound = 0;
    Beacon myBeacon;
    Beacon beaconTab[3];
    Beacon bestBeaconTab[3];
    double object_width = 0;
    uint8_t already_added = 0;
    for (int i=1; i<counter;++i){
        actualDistance = distance(refa, a[i],refd, d[i]);

        if(actualDistance <= objectMaxStep){ //Si c'est le même objet on fait une moyenne
        //if(d[i]-refd <=0.13){
            //printf("a[i] %f \n", a[i]);
            moyd+=d[i];
            moya+=a[i];
            //printf("moya %d \n", moya);
            //printf("moyd %d \n", moyd);
            moy_count+=1;
            object_width += distance(a[i],refa,d[i],refd);
            refa=a[i];
            refd=d[i];

        }
        else{ //l'objet est fini, on l'ajoute a la liste si il n'est pas trop grand

            if(object_width < max_object_width /*&& object_width != 0*/){
                newa.push_back(moya/float(moy_count));
                newd.push_back(moyd/float(moy_count));
                newWidth.push_back(object_width);
                //if(verbose) fprintf(stderr,"object added with moya = %f\ at distance %f  and width %f\n",moya/float(moy_count),moyd/float(moy_count),object_width);
                obj_iter+=1;
            }
            
            
            refa=a[i];
            refd=d[i];
            moyd=refd;
            moya=refa;
            moy_count=1;
            
            object_width = 0;

        }
        if(i == counter-1 && object_width < 0.3){

            if(distance(newa[0],moya/moy_count,newd[0],moyd/moy_count) < objectMaxStep){
                if(verbose) fprintf(stderr,"Object fusion engaged \n");
                if(verbose) fprintf(stderr,"distance for fusion is %f \n",distance(newa[0],moya/moy_count,newd[0],moyd/moy_count));
                        newa[i] = (newa[0]*newWidth[i]+(moya/moy_count -360) *object_width)/(object_width+newWidth[0]);
                        if(newa[i] < 0) newa[i] = newa[i] + 360;
                        //newd[i] = (newd[i]*newWidth[i]+(moyd/moy_count) *object_width)/(object_width+newWidth[i]);
                        newWidth[0] = newWidth[0] + object_width;
                        already_added = 1;
            }
            if(!already_added){
            if(verbose) fprintf(stderr,"object added with moya = %f\ at distance %f  and width %f\n",moya/float(moy_count),moyd/float(moy_count),object_width);
            newa.push_back(moya/float(moy_count));
            newd.push_back(moyd/float(moy_count));
            newWidth.push_back(object_width);
            }
            refa=a[i];
            refd=d[i];
            moyd=refd;
            moya=refa;
            moy_count=1;
            obj_iter+=1;
        }

    }
    /*for(int i = 0; i< newa.size();i++){
        if(verbose) fprintf(stderr,"Objet trouve à angle %f et distance %f \n",newa[i],newd[i]);
    }*/
    pthread_mutex_lock(&lockMyState);
    if(myState == CALIB_MODE){
        
        for(int h = 0; h< newa.size();h++){
            if(newd[h] > 3.15-isoceleTolerance && newd[h] < 3.15+isoceleTolerance && newa[h] < 15.85+4 && newa[h] > 15.85-4){
                myPos.theta = atan((beaconRefPosition[1].x-calibPos.x)/(beaconRefPosition[1].y-calibPos.y));
                beaconRefPosition[1].x = calibPos.x + newd[h] * cos((myPos.theta - newa[h] + 90)*DEG2RAD);
                beaconRefPosition[1].y = calibPos.y + newd[h] * sin((myPos.theta - newa[h] + 90)*DEG2RAD);
            }}
            for(int h = 0; h< newa.size();h++){
                if(newd[h] > 0.20-isoceleTolerance && newd[h] < 0.2+isoceleTolerance && newa[h] < 180+22+4 && newa[h] > 180+22-4){
                    beaconRefPosition[0].x = calibPos.x + newd[h] * cos((myPos.theta - newa[h] + 90)*DEG2RAD);
                    beaconRefPosition[0].y = calibPos.y + newd[h] * sin((myPos.theta - newa[h] + 90)*DEG2RAD);
                }
                
                if(newd[h] > 1.85-isoceleTolerance && newd[h] < 1.85+isoceleTolerance && newa[h] < 96.3+4 && newa[h] > 96.3-4){
                    beaconRefPosition[2].x = calibPos.x + newd[h] * cos((myPos.theta - newa[h] + 90)*DEG2RAD);
                    beaconRefPosition[2].y = calibPos.y + newd[h] * sin((myPos.theta - newa[h] + 90)*DEG2RAD);
                }
                
                
                
            }
            perimetre = beaconDistance(beaconRefPosition[0],beaconRefPosition[1]) + beaconDistance(beaconRefPosition[1],beaconRefPosition[2])+ beaconDistance(beaconRefPosition[2],beaconRefPosition[0]);
                fprintf(stderr,"perimetre = %f \n",perimetre);
            fprintf(stderr,"Position des balises calibrée: x1 = %f y1 = %f x2 = %f y2 = %f x3 = %f y3 = %f",beaconRefPosition[0].x,beaconRefPosition[0].y,beaconRefPosition[1].x,beaconRefPosition[1].y,beaconRefPosition[2].x,beaconRefPosition[2].y);
            myState = LOCALIZE_MODE;
    }
    pthread_mutex_unlock(&lockMyState);
    
    //(float[2]) coord[3]={};
    //int coord[3]={};
    std::vector<std::vector<float>> balises (3,std::vector<float>(2));
    for (int i = 0; i < obj_iter; i++) //Boucle pour trouver balises
    {
	//std::cout<<"il rentre dans la boucle?";
        float a1=newa[i];
        float d1=newd[i] +baliseRadius;
        float w1 = newWidth[i];
        for (int j = i+1; j<obj_iter; j++)//on va dans ce sens là pour aller plus vite
        {
	    //std::cout<<"et dans celle ci aussi?";
            float a2=newa[j];
            float d2=newd[j] +baliseRadius;
            float w2 = newWidth[j];
            for (int k = j+1; k<obj_iter; k++)
            {
                float a3=newa[k];
                float d3=newd[k] +baliseRadius;
                float w3 = newWidth[k];
                float triangle= distance(a1,a2,d1,d2)+distance(a1,a3,d1,d3)+distance(a2,a3,d2,d3); //sensé être 3+2.5+2.5 donc 8
                //printf("triangle: %f, i: %d, j: %d, k: %d \n", triangle, i, j, k);
		//printf("ai: %f, aj: %f, ak: %f \n", newa[i], newa[j], newa[k]);
                float dij=distance(a1,a2,d1,d2);
                float djk=distance(a2,a3,d2,d3);
                float dik=distance(a1,a3,d1,d3);
                
                
                
                
                //if(triangle<=8 && triangle>=7.8 && dij<=3.3 && dij>=1.8 && djk<=3.3 && djk>=1.8 && dik<=3.3 && dik>=1.8 && (newd[i]+newd[j]<=6.6 && newd[k]+newd[j]<=6.6) && (newa[j]-newa[i])>=30.0 && (newa[k]-newa[j])>=30.0){//faudrait rajouter une condition brrr genre sur les anngles
                    //Ici c'est là où j'ai changé
                
                //SORT LES BALISES
                
                beaconTab[0].distance = d1; beaconTab[0].angle = a1; beaconTab[0].width = w1;
                beaconTab[1].distance = d2; beaconTab[1].angle = a2; beaconTab[1].width = w2;
                beaconTab[2].distance = d3; beaconTab[2].angle = a3; beaconTab[2].width = w3;
                int n = 3;
                
                Beacon tempoBeacon;

                //VRAI TRI SA MERE POUR REMETTRE BALISES DANS LORDRE
                while(distance(beaconTab[0].angle, beaconTab[1].angle,beaconTab[0].distance, beaconTab[1].distance) < distance(beaconTab[0].angle, beaconTab[2].angle,beaconTab[0].distance, beaconTab[2].distance) || distance(beaconTab[1].angle, beaconTab[2].angle,beaconTab[1].distance, beaconTab[2].distance)<distance(beaconTab[0].angle, beaconTab[2].angle,beaconTab[0].distance, beaconTab[2].distance)){
                    tempoBeacon = beaconTab[1];
                    beaconTab[1] = beaconTab[0];
                    beaconTab[0] = beaconTab[2];
                    beaconTab[2] = tempoBeacon;
                }
                    
                    dij=distance(beaconTab[0].angle,beaconTab[1].angle,beaconTab[0].distance,beaconTab[1].distance);
                    djk=distance(beaconTab[1].angle,beaconTab[2].angle,beaconTab[1].distance,beaconTab[2].distance);
                    dik=distance(beaconTab[0].angle,beaconTab[2].angle,beaconTab[0].distance,beaconTab[2].distance);
                    //fprintf(stderr,"djk = %f\n",djk);
                    
                    
                    float actualError;
                    pthread_mutex_lock(&filteredPositionLock);
                    // BeaconSelection
                    double dx0 = beaconRefPosition[0].x - filteredPos.x;
                    double dy0 = beaconRefPosition[0].y - filteredPos.y;
                    double dx1 = beaconRefPosition[1].x -filteredPos.x;
                    double dy1 = beaconRefPosition[1].y - filteredPos.y;
                    double dx2 = beaconRefPosition[2].x -filteredPos.x;
                    double dy2 = beaconRefPosition[2].y - filteredPos.y;

                    double angle_balise_robot0 = atan2(dx0, dy0) * 180.0 / M_PI; // Angle entre le robot et la balise dans le sens trigonométrique
                    double angle_balise_robot1 = atan2(dx1, dy1) * 180.0 / M_PI; // Angle entre le robot et la balise dans le sens trigonométrique
                    double angle_balise_robot2 = atan2(dx2, dy2) * 180.0 / M_PI; // Angle entre le robot et la balise dans le sens trigonométrique

                    double angle_robot_trigo = filteredPos.theta; // Angle du robot par rapport à l'axe y dans le sens trigonométrique

                    // Conversion de l'angle du sens trigonométrique au sens horlogique
                    // Conversion de l'angle du sens trigonométrique au sens horlogique
                    double angle_balise_robot_horloge0 = fmod((360 - angle_balise_robot0), 360);
                    double angle_balise_robot_horloge1 = fmod((360 - angle_balise_robot1), 360);
                    double angle_balise_robot_horloge2 = fmod((360 - angle_balise_robot2), 360);

                    double angle_robot_trigo_horloge = fmod((360 - angle_robot_trigo), 360);

                    pthread_mutex_unlock(&filteredPositionLock);

                    // Calcul de l'angle entre le robot et la balise dans le sens horlogique
                    double angle_horlogique0 = 360-fmod((angle_balise_robot_horloge0 + angle_robot_trigo_horloge), 360);
                    if(angle_horlogique0<0) angle_horlogique0+=360;
                    if(angle_horlogique0 > 360) angle_horlogique0+=-360;

                    double angle_horlogique1 = 360-fmod((angle_balise_robot_horloge1 + angle_robot_trigo_horloge), 360);
                    if(angle_horlogique1<0) angle_horlogique1+=360;
                    if(angle_horlogique1 > 360) angle_horlogique1+=-360;

                    double angle_horlogique2 = 360-fmod((angle_balise_robot_horloge2 + angle_robot_trigo_horloge), 360);
                    if(angle_horlogique2<0) angle_horlogique2+=360;
                    if(angle_horlogique2 > 360) angle_horlogique2+=-360;

                    //printf("Angle horlogique = %f %f %f \n" ,angle_horlogique0, angle_horlogique1, angle_horlogique2);

                    //printf("angle horlogique = %f %f %f et beaconRefPosition = %f %f  ; %f %f ; %f %f et angle baliseRobot = %f\n dx = %f et dy = %f \n",angle_horlogique0,angle_horlogique1,angle_horlogique2,beaconRefPosition[0].x,beaconRefPosition[0].y,beaconRefPosition[1].x,beaconRefPosition[1].y,beaconRefPosition[2].x,beaconRefPosition[2].y, angle_balise_robot0,dx0,dy0);
                    //printf("Position actuelle = %f %f %f \n",filteredPos.x,filteredPos.y,filteredPos.theta);


                    uint8_t condition = (beaconTab[0].angle > (angle_horlogique0 - angleTolerance) && beaconTab[0].angle < (angle_horlogique0 + angleTolerance)) && (beaconTab[1].angle > (angle_horlogique1 - angleTolerance) && beaconTab[1].angle < (angle_horlogique1 + angleTolerance)) && (beaconTab[2].angle > (angle_horlogique2 - angleTolerance) && beaconTab[2].angle < (angle_horlogique2 + angleTolerance))  && triangle<=perimetre+triangleErrorTolerance && triangle>=perimetre-triangleErrorTolerance && dij<=longSideLength+isoceleTolerance && dij>=longSideLength-isoceleTolerance && djk<=longSideLength+isoceleTolerance && djk>=longSideLength-isoceleTolerance && dik>=shortSideLength-isoceleTolerance && dik<=shortSideLength+isoceleTolerance;
                    actualError = fabs(triangle-perimetre) + fabs(fabs(dij-djk));
                    //DEBUG ---------------
                        debugBalises[0][0] = angle_horlogique0;
                        debugBalises[1][0] = angle_horlogique1;
                        debugBalises[2][0] = angle_horlogique2;
                    //FIN-DEBUG -----------
                    
                    //if(beaconTab[0].angle >270 && beaconTab[0].angle < 320 && triangle > 8.35 && triangle < 8.45)        if(verbose) fprintf(stderr," distances: %f %f %f angles: %f %f %f périmètre: %f \n conditions: %d %d %d %d %d %d %d %d \n",beaconTab[0].distance, beaconTab[1].distance,beaconTab[2].distance,beaconTab[0].angle, beaconTab[1].angle,beaconTab[2].angle,triangle, triangle<=perimetre+triangleErrorTolerance , triangle>=perimetre-triangleErrorTolerance , dij<=3.25+isoceleTolerance , dij>=3.25-isoceleTolerance , djk<=3.25+isoceleTolerance ,djk<=3.25-isoceleTolerance , dik>=1.9-isoceleTolerance , dik<=1.9+isoceleTolerance);
                    //if(verbose && triangle >8.5&& triangle < 8.7 /*&& fabs(beaconTab[0].angle-90) < 20*/) printf("Nous avons un triangle de taille %f à angles %f %f %f à une distance %f %f %f %d %d %d %d %d %d %d %d \n",triangle,a1,a2,a3, dij,djk,dik, triangle<=perimetre+triangleErrorTolerance , triangle>=perimetre-triangleErrorTolerance , dij<=3.2+isoceleTolerance , dij>=3.2-isoceleTolerance , djk>=3.2-isoceleTolerance , djk<=3.2+isoceleTolerance , dik>=2-isoceleTolerance , dik<=2+isoceleTolerance);
                    if(condition && actualError < oldError){//faudrait rajouter une condition brrr genre sur les anngles

                        //DEBUG --------
                        debugIsoceleCondition = dij - djk;
                        debugPerimetre = triangle;
                        
                        //FIN-DEBUG --------

                        beaconFound = 1;
                        bestBeaconTab[0] = beaconTab[0];
                        bestBeaconTab[1] = beaconTab[1];
                        bestBeaconTab[2] = beaconTab[2];

                        oldPerimetre = triangle;
                        oldIsoceleCondition = fabs(dij-djk);
                        oldError = actualError;
                        if(verbose){
                            //fprintf(stderr,"Dij = %f Djk = %d Djk = %f",dij,djk,dik);
                        }
                        
                    
                        }

                        

                    }
                    
                }
            
            }
            if(beaconFound){
                //printf("angle horlogique = %f %f %f et beaconRefPosition = %f %f  ; %f %f ; %f %f et angle baliseRobot = %f\n dx = %f et dy = %f \n",angle_horlogique0,angle_horlogique1,angle_horlogique2,beaconRefPosition[0].x,beaconRefPosition[0].y,beaconRefPosition[1].x,beaconRefPosition[1].y,beaconRefPosition[2].x,beaconRefPosition[2].y, angle_balise_robot0,dx0,dy0);
                //fprintf(stderr,"\n Balises: (%f,%f) width = %f, (%f, %f) width = %f, (%f, %f) width = %f perimetre = %f, dij = %f, djk = %f dik = %f  \n",bestBeaconTab[0].angle,bestBeaconTab[0].distance,bestBeaconTab[0].width,bestBeaconTab[1].angle,bestBeaconTab[1].distance,bestBeaconTab[1].width,bestBeaconTab[2].angle,bestBeaconTab[2].distance,bestBeaconTab[2].width, oldPerimetre,distance(bestBeaconTab[0].angle,bestBeaconTab[1].angle,bestBeaconTab[0].distance,bestBeaconTab[1].distance),distance(bestBeaconTab[1].angle,bestBeaconTab[2].angle,bestBeaconTab[1].distance,bestBeaconTab[2].distance),distance(bestBeaconTab[0].angle,bestBeaconTab[2].angle,bestBeaconTab[0].distance,bestBeaconTab[2].distance) );
                if(verbose) printf("triangle: %f \n", oldPerimetre);
                pthread_mutex_lock(&positionLock);
                *myX = 0; *myY = 0;
                
            
            pthread_mutex_unlock(&positionLock);
            triangulationPierlot( myX,  myY, (360-bestBeaconTab[0].angle)*DEG2RAD,  (360-bestBeaconTab[1].angle)*DEG2RAD,  (360-bestBeaconTab[2].angle)*DEG2RAD, beaconRefPosition[0].x, beaconRefPosition[0].y,beaconRefPosition[1].x, beaconRefPosition[1].y,beaconRefPosition[2].x, beaconRefPosition[2].y);
            //fprintf(stderr,"myX = %f and myY = %d \n", myX,myY);
            //Calcul angle augustin
            float alpha;
            //if(myColor == BLUE) alpha = 180-(360-bestBeaconTab[0].angle) - atan(*myX/(*myY))*RAD2DEG;
            if(myColor == BLUE) alpha = 180-(360-bestBeaconTab[0].angle) - atan((*myX - beaconRefPosition[0].x) / (*myY - beaconRefPosition[0].y))*RAD2DEG;
            else alpha = 360-((360 - bestBeaconTab[0].angle) + atan((*myX - beaconRefPosition[0].x) / (*myY - beaconRefPosition[0].y)) * RAD2DEG);

            if(alpha<0) alpha = alpha+360;
            //if(verbose) fprintf(stderr,"angle = %f \n",alpha);
            
            pthread_mutex_lock(&positionLock);
            //fprintf(stderr,"myX = %f and myY = %d \n", *myX,*myY);
            myPos.x = *myX;
            myPos.y = *myY;
            //fprintf(stderr,"position: x = %f y = %f theta = %f \n",*myX,*myY,alpha);
            
            //if(verbose) fprintf(stderr,"beacon data\n");
            myPos.theta = alpha;
            
            pthread_mutex_unlock(&positionLock);
            }

            if(filteredPosAcquired){
            float tempoX, tempoY, tempoA, tempoD = 0;
            bool opponentFound = 0;
            lidarPos object;
            pthread_mutex_lock(&lockOpponentPosition);
            pthread_mutex_lock(&filteredPositionLock);
            float previousDistanceToCenter = 3;
            float previousD = 3;
            float score, previousScore = 0;
            for (int k = 0;k<newa.size();k++){
                    object.x = filteredPos.x + newd[k] * cos((filteredPos.theta - newa[k] + 90)*DEG2RAD);
                    object.y = filteredPos.y + newd[k] * sin((filteredPos.theta - newa[k] + 90)*DEG2RAD);
                    //object.x = filteredPos.x + newd[k] * cos((90-(360-newa[k]-filteredPos.theta))*DEG2RAD);
                    //object.y = filteredPos.y + newd[k] * sin((90-(360-newa[k]-filteredPos.theta))*DEG2RAD);


                    objects_coordinates.push_back(object);
                    //fprintf(stderr,"object position: x = %f and y = %f angle = %f\n",object.x,object.y,newa[k]);
                    distanceToCenter = pow((object.x-1)*(object.x-1)+(object.y-1.5)*(object.y-1.5),0.5);

                    //printf("distanceToCenter = %f\n",distanceToCenter);
                    score = 3/(distanceToCenter+0.5) + 2/(newd[k]+0.5);
                    if(object.x < 1.87 && object.x > 0.13 && object.y < 2.87 && object.y > 0.13 && score>previousScore){
                        //printf("PotentialOpponent at %f %f \n",object.x,object.y);
                        //if(verbose) fprintf(stderr, "opponent added \n");
                        tempoX = object.x;
                        tempoY = object.y;
                        tempoA = newa[k];
                        tempoD = newd[k];
                        myOpponent.isDetected = 1;
                        previousDistanceToCenter = distanceToCenter;
                        opponentFound = 1;
                        previousScore = score;

                    }
                }
            if(opponentFound){
                    if(tempoD < 0.3) printf("chosen Opponent = %f %f \n",tempoX,tempoY);
                    myOpponent.x = tempoX;
                    myOpponent.y = tempoY;
                    
                }
                writeLog(tempoA,tempoD,previousDistanceToCenter,debugBalises,debugPerimetre,debugIsoceleCondition);
            pthread_mutex_unlock(&filteredPositionLock);
            pthread_mutex_unlock(&lockOpponentPosition);
            }

    //fprintf(stderr,"Après giga boucle beacon_data  \n");
    /*pthread_mutex_lock(&positionLock);
    myPos.x = 0.0;
    myPos.y = 0.0;
    pthread_mutex_unlock(&positionLock);*/
    //if(verbose) fprintf(stderr,"beacon data\n");
    //pthread_mutex_lock(&isReadyLock);
    //readyToSend = 1;
    //pthread_mutex_unlock(&isReadyLock);
    
    return NULL;
}

int read_fd;


int main(int argc, const char * argv[]){
    pthread_mutex_lock(&lockMyState);
    

    calibPos.x = 0.13;
    calibPos.y = 0.125;
    calibPos.theta = 0;
    
    myState = LOCALIZE_MODE;
    //myState = CALIB_MODE;
    verbose =1;
    myOpponent.isDetected = 0;
    pthread_mutex_unlock(&lockMyState);
    if(verbose) fprintf(stderr,"Argc  = %d\n",argc);
    int write_fd;
    
    myX = (float*) safe_malloc(sizeof(float),1);
    myY = (float*) safe_malloc(sizeof(float),2);
    pthread_t acquisitionThread;
    myColor = BLUE;
    if(argc > 1){ 
        write_fd = atoi(argv[1]); // Récupération du descripteur de fichier d'écriture du pipe à partir des arguments de la ligne de commande
        read_fd = atoi(argv[2]);
        startPosition = atoi(argv[3]);

        pthread_mutex_lock(&filteredPositionLock);
        if(startPosition == 1) {
            filteredPos.x = 0.2;
            filteredPos.y = 0.2;
            filteredPos.theta = 0;
            myColor = BLUE;
        }
        else if(startPosition == 2) {
            filteredPos.x = 1;
            filteredPos.y = 2.8;
            filteredPos.theta = 180;
            myColor = BLUE;
        }
        else if(startPosition == 3) {
            filteredPos.x = 1.8;
            filteredPos.y = 0.2;
            filteredPos.theta = 0;
            myColor = BLUE;
        }
        else if(startPosition == 4) {
            filteredPos.x = 0.2;
            filteredPos.y = 2.8;
            filteredPos.theta = 180;
            myColor = YELLOW;
        }
        else if(startPosition == 5){
            filteredPos.x = 1;
            filteredPos.y = 0.2;
            filteredPos.theta = 0;
            myColor = YELLOW;
        }
        else if(startPosition == 6){
            filteredPos.x = 1.8;
            filteredPos.y = 2.8;
            filteredPos.theta = 180;
            myColor = YELLOW;
        }
        pthread_mutex_unlock(&filteredPositionLock);

        printf("startingPoint = %d and initialPosition = %f %f %f and team = %d\n",startPosition,filteredPos.x,filteredPos.y,filteredPos.theta,myColor);

        if(myColor == BLUE){
            beaconRefPosition[0].x = 0.05;
            beaconRefPosition[0].y = -0.08;
            beaconRefPosition[1].x = 1;
            beaconRefPosition[1].y= 3.08;
            beaconRefPosition[2].x = 1.95;
            beaconRefPosition[2].y = -0.08;
        }
        else{
            beaconRefPosition[0].x = 1.95;
            beaconRefPosition[0].y = 3.08;
            beaconRefPosition[1].x = 1;
            beaconRefPosition[1].y= -0.08;
            beaconRefPosition[2].x = 0.05;
            beaconRefPosition[2].y = 3.08;
        }

        
        
    

        verbose = 0;
        generateLog();
        
        if (pthread_create(&acquisitionThread, NULL, filteredPositionAcquisition, NULL) != 0) {
        fprintf(stderr, "Erreur lors de la création du thread\n");
        exit(EXIT_FAILURE);
    }
        
        }
    lidarPos *position = &myPos;
    //clock_t begin= clock();
    ///  Create a communication channel instance
    IChannel* _channel;//oskur il connait pas ça
    Result<IChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);//on crée le canal pour le port, on a un usb donc c'est good (checker la valeur mais c'est good normalement)
    //std::cout << "found the lidar at the port 115200";
    ILidarDriver * lidar = *createLidarDriver();//le driver
    auto res = (*lidar).connect(*channel);//on connecte le driver au channel
    if(SL_IS_OK(res)){//si ça marche
	//std::cout<<"et ici ça passe?";
	sl_lidar_response_device_info_t deviceInfo;//récupérer les infos de l'appareil
        res = (*lidar).getDeviceInfo(deviceInfo);//on les stocke la
        if(SL_IS_OK(res)){//si ça va
		//std::ifstream file;
		//file.open("lidar_bord_g_vers2.txt");
		printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",//print les donner
		deviceInfo.model,
		deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
		deviceInfo.hardware_version);
		//sleep(5);
		//lidar->setMotorSpeed(0); ça change rien c'est bizarreee
		std::vector<LidarScanMode> scanModes;  // ça c'est si on veut choisir le mode de scan
		lidar->getAllSupportedScanModes(scanModes);
		//lidar->startScanExpress(false, scanModes[0].id);
		//lidar->setMotorSpeed(1);

		LidarScanMode scanMode;//on utilise le mode standard de scan(on peut aussi choisir)
		lidar->startScan(false, true, 0, &scanMode);

		sl_lidar_response_measurement_node_hq_t nodes[8192];//on définit un format de réponse
		size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);//on définit la taille du truc
        pthread_t computationThread;
        lidar_data myLidarData;
        int thread_launched = 0;
        sl_result res_gscan;
        		    float angle[8192];
            //if(verbose) fprintf(stderr,"Check 1\n");
		    float distance[8192];
        while(1){
        res_gscan = lidar->grabScanDataHq(nodes, nodeCount);//on remplit avec le grab data (ici hq pas nécéssaire, <16m)
        //res_gscan= lidar->getScanDataWithIntervalHq(nodes, nodeCount);//faudrait checker la diff avec le continu
        
		
		if (IS_OK(res_gscan)){
            
		    //if(verbose) fprintf(stderr, "Hey mais... le grabscan marche");//erreur si je sais pas grab les data
		    lidar->ascendScanData(nodes, nodeCount);
            
		    //std::ofstream out("lidar_bord_g_vers2.txt");
            /*if(thread_launched){
            while(1){
        ;
    }}*/
            if(thread_launched) {
                if(pthread_join(computationThread, NULL) != 0){ fprintf(stderr,"error while joining thread \n");} //Attends que le dernier calcul soit fini avant d'en lancer un nouveau
                //thread_launched = 0;
                readyToSend = 1;
                //fprintf(stderr,"thread joined \n");
            }

            
            pthread_mutex_lock(&lidarDataLock);

		    int counter=0;
            
		    for(int i=0;i<(int)nodeCount;i++){
                //if(verbose) fprintf(stderr,"Check 2\n");
			float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
			float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
			//out << angle_in_degrees << " , " << distance_in_meters << "\n";
                //if(verbose) fprintf(stderr,"Check 3\n");
            
			if(distance_in_meters<=limit_of_detection && distance_in_meters!=0.0){
			    angle[counter]=angle_in_degrees;
			    distance[counter]=distance_in_meters;
			    counter+=1;
			    //out << angle_in_degrees << " , " << distance_in_meters << "\n";
			    //printf("Angle : %f, Distance : %f \n", angle_in_degrees,distance_in_meters);
			}
			
		    }
            //if(verbose) fprintf(stderr,"Check 4\n");
            
            lidarDataCopied = 0;
            myLidarData.distance = distance;
            myLidarData.angle = angle;
            myLidarData.counter = counter;
            if(verbose) fprintf(stderr,"counter in main = %d \n",myLidarData.counter);
            //for(int l = 0; l < counter; l++) if(myLidarData.angle[l]>360) //fprintf(stderr,"dans main angle = %f \n",myLidarData.angle[l]);
            
            int thread_execut = pthread_create(&computationThread, NULL, beacon_data, (void*) &myLidarData);
            if(verbose)fprintf(stderr,"passe après le thread thread code = %d\n",thread_execut);
            if (thread_execut != 0) {
                if(verbose) fprintf(stderr, "Erreur lors de la création du thread.\n");
                return EXIT_FAILURE;
            }
    
            thread_launched = 1;


    
    
    
    pthread_mutex_unlock(&lidarDataLock);
    }
else{
        if(verbose) fprintf(stderr, "OSKUR poupon, failed to grab scan the data with LIDAR %08x\r\n", res_gscan);//erreur si je sais pas grab les data
        //return -1;
        //ici faut recup les donner de res scan
    }
    
    

   
    if(argc >1){
    //if(verbose) fprintf(stderr,"Check 6,5\n");
    pthread_mutex_lock(&isReadyLock);
    if(readyToSend){
        pthread_mutex_lock(&positionLock);
        std::vector<float> numbers = {position->x,position->y,position->theta,myOpponent.x,myOpponent.y}; // Déclaration du tableau de nombres à virgule flottante à envoyer
        pthread_mutex_unlock(&positionLock);
        //if(verbose) fprintf(stderr,"Check 7\n");
        write(write_fd, numbers.data(), numbers.size() * sizeof(float)); // Écriture des nombres dans le pipe
    }
    pthread_mutex_unlock(&isReadyLock);
    
    
    
    
    
    }
    
    pthread_mutex_lock(&positionLock);
    pthread_mutex_lock(&lockOpponentPosition);
    if(verbose){
     fprintf(stderr,"position x = %f \n",position->x);
     fprintf(stderr,"position y = %f \n",position->y);
     fprintf(stderr,"position theta = %f \n",position->theta);
    fprintf(stderr,"position x opponent = %f \n",myOpponent.x);
     fprintf(stderr,"position y opponent = %f \n",myOpponent.y);
    }

    pthread_mutex_unlock(&positionLock);
    pthread_mutex_unlock(&lockOpponentPosition);
    
    //if(verbose) fprintf(stderr,"Check 8\n");
    
    //if(verbose) sleep(2);


    
    
    }
        
	//sleep(1);
    }else{
        if(verbose) fprintf(stderr, "OSKUR poupon, failed to get device information from LIDAR %08x\r\n", res);
    }


    //sleep(5);
    //lidar->setMotorSpeed(1);

}else{
    if(verbose) fprintf(stderr, "OSKUR poupon, Failed to connect to LIDAR %08x\r\n", res);
}
    //clock_t end= clock();
    //double time_spent= (double)(end-begin)/CLOCKS_PER_SEC;
    //printf("execution time: %f \n", time_spent);
    //close(write_fd); // Fermeture du descripteur de fichier d'écriture du pipe
    pthread_join(acquisitionThread,NULL);
}

void* filteredPositionAcquisition(void* arg){
    struct timeval timeout;
    timeout.tv_sec = 0; // Timeout de 1 seconde pour la reception de la position filtrée
    timeout.tv_usec = 3000;
    fd_set set;
    

    while(1){
        FD_ZERO(&set); // Initialiser le set à zéro
    FD_SET(read_fd, &set); // Ajouter le descripteur de fichier de lecture du pipe au set
    
    int ret = select(read_fd + 1, &set, NULL, NULL, &timeout);
    if(verbose) printf("ret = %d \n",ret);
    if (ret == -1) {
            perror("select");
            exit(EXIT_FAILURE);
        } else if (ret == 0) {
            //printf("No data within one seconds.\n");
        } //DES DONNEES SONT DISPONIBLES
        else{
            float buffer[3];
            read(read_fd, buffer, sizeof(buffer));
            pthread_mutex_lock(&filteredPositionLock);
            filteredPos.x = buffer[0];
            filteredPos.y = buffer[1];
            filteredPos.theta = buffer[2];
            pthread_mutex_unlock(&filteredPositionLock);
            filteredPosAcquired = true;
            if(verbose) printf("BUFFER SETTED and position filtered is %f %f %f \n",buffer[0],buffer[1],buffer[2]);
        }
    }
}

void generateLog(){
    int variable = 0;

    logFile = fopen("../logFiles/logLidar.txt", "w");
    if (logFile == NULL) {
        printf("Erreur lors de l'ouverture du fichier LogLidar\n");
    }
    fprintf(logFile, "myFilteredPos ; myLidarPos ; opponentPos ; opponentAngle ; opponentDistance ; DistanceFromCenter ; Angle balises recherchés ; debugPerimetre ; debugIsocele \n");
    printf("File generated i guess\n");
}

void writeLog(float angle, float distance, float distanceFromCenter, float debugBalises[3][2], float debugPerimetre, float debugIsoceleCondition){
    fprintf(logFile, "%f %f %f ; %f %f %f ; %f %f ; %f ; %f ; %f ; %f %f %f ; %f ; %f \n",filteredPos.x, filteredPos.y, filteredPos.theta,myPos.x,myPos.y,myPos.theta,myOpponent.x,myOpponent.y,angle, distance, distanceFromCenter,debugBalises[0][0],debugBalises[1][0],debugBalises[2][0],debugPerimetre,debugIsoceleCondition);
}