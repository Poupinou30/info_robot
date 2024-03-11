//#include <math.h>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include "lidar.h"
#include <rplidar.h>
#include <iostream>
#include <fstream>

//#include "matplotlibcpp.h"
using namespace sl;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

//Juste un petit test git

typedef struct lidarPos{
    float x;
    float y;
    float theta;
} lidarPos;
lidarPos myPos;
int w_plot(float x[], float y[], float angle[], float distance[], int length) {//x c'est angle

  FILE *gnuplot = popen("gnuplot -persist", "w");
	if (!gnuplot) {
	    perror("popen");
	    exit(EXIT_FAILURE);
	}
  
  fprintf(gnuplot,"set xlabel 'x'\n");
  fprintf(gnuplot,"set ylabel 'y'\n");
  fprintf(gnuplot, "plot '-' using 1:2 with points lc rgb 'blue', \'-' using 1:2 with points lc rgb 'orange', \'-' using 1:2 with points lc rgb 'red' \n");
  //fprintf(gnuplot, "plot 'balises' using 1:2 with points lc rgb 'green' \n");
  
  for (int i = 0; i < 3; ++i) {
    //if(y[i]*cos(x[i]*(M_PI/180.0))<3.0 && y[i]*cos(x[i]*(M_PI/180.0)>-3.0))
    fprintf(gnuplot, "%f %f \n", distance[i]*cos(-angle[i]*(M_PI/180.0)), distance[i]*sin(-angle[i]*(M_PI/180.0)));
    //fprintf(gnuplot, "%d %d\n", x[i], y[i]);
  }
  fprintf(gnuplot,  "e\n");
  //fprintf(gnuplot, "replot 'data' using 1:2 with points lc rgb 'blue' \n");
  for (int i = 0; i < length; ++i) {
    //if(y[i]*cos(x[i]*(M_PI/180.0))<3.0 && y[i]*cos(x[i]*(M_PI/180.0)>-3.0))
    fprintf(gnuplot, "%f %f \n", y[i]*cos(-x[i]*(M_PI/180.0)), y[i]*sin(-x[i]*(M_PI/180.0)));
    //fprintf(gnuplot, "%d %d\n", x[i], y[i]);
  }
  fprintf(gnuplot,  "e\n");
  //fprintf(gnuplot, "replot 'robot' using 1:2 with points lc rgb 'red'\n");
  double r=0.1;
  double x0,y0;
  double y1,x1;
  for(y1=y0-r;y1<=y0+r;y1=y1+0.1){
    x1=sqrt(r*r-(y1-y0)*(y1-y0))+x0; 
    fprintf(gnuplot,"%lf\t %lf l\n",x1,y1);
  }
  for(y1=y0+r;y1>=y0-r;y1=y1-0.1){
    x1=-sqrt(r*r-(y1-y0)*(y1-y0))+x0; 
    fprintf(gnuplot,"%lf\t %lf l\n",x1,y1);
  }
  fprintf(gnuplot, " %f %f \n",0.0,0.0);
  fprintf(gnuplot, "e\n");
  //fprintf(stdout, "Click Ctrl+d to quit...\n");
  fflush(gnuplot);
  getchar();

  pclose(gnuplot);
  //exit(EXIT_SUCCESS);

}
double distance (double a1,double a2,double d1,double d2){
    double dist= sqrt(pow(d1,2)+pow(d2,2) - (2*d1*d2*cos((a1-a2)*M_PI/180)));
    //fprintf(stderr,"distance is %lf",dist);
    return dist;
}
float pyth_gen(float a, float b, float c){
    //printf("test:%f", pow(b,2));
    float cos= (pow(b,2)+pow(c,2)-pow(a,2))/2*b*c;
    //printf("test:%f", acos(cos));
    return acos(cos);
}

std::vector<std::vector<float>> detect_obstacle(std::vector<float> newa ,std::vector<float> newd,int counter){
    std::vector<std::vector<float>> obstacles(2);
    std::vector<float> obstacle_a;
    std::vector<float> obstacle_d;
    //std::cout<<"tu entres ici ouhouuuuu";
    for(int i=0; i<counter;i++){
	if(newd[i]<0.3){
	    obstacle_a.push_back(newa[i]);
	    obstacle_d.push_back(newd[i]);
	    printf("ATTENTION ATTENTION OBSTACLE/SIMON à %f degrés", newa[i]);
	}
	else{
	    //std::cout<<"Pas d'obstacle dans le champ";
	    obstacle_a.push_back(0);
	    obstacle_d.push_back(0);
	}
    }
    obstacles[0]=obstacle_a;
    obstacles[1]=obstacle_d;
    return obstacles;

}
float angle_robot(std::vector<std::vector<float>> balises){
    /*
     *
     * le but est de retourner l'angle du robot par rapport à l'orientation de la table
     * on va faire un triangle formé du robot et des 2 balises du même coté et utilisé pythagore généralisé
     */
     //Attention à partir d'ici les balises sont en radians
    float yRm = myPos.y;
    float xRm = myPos.x;
    printf("yRm: %f , balises[1][0] : %f, rapport: %f", yRm, balises[1][0], yRm/balises[1][1]);
    //printf("test pyth_gen %f", pyth_gen(balises[2][1], balises[0][1], 2.0));
    
    
    //float yAli= balises[0][1]*sin((M_PI/180.0)*pyth_gen(balises[2][1], balises[0][1], 2.0));
    //float xAli= balises[0][1]*cos((M_PI/180.0)*pyth_gen(balises[2][1], balises[0][1], 2.0));
    
    float theta=balises[1][0]-acosf(yRm/balises[1][1]);//acos il sort du radian donc c'est ok vu que balises[1][0] est en radians aussi
    theta= theta*180.0/M_PI;//ici j'ai juste mis cette ligne ci sinon on était en radian
    printf("theta: %f, xRobotMan = %f, yRobotMan=%f",theta, xAli, yAli);
    return theta;
}

lidarPos beacon_data(float a[] ,float d[],int counter){
    /*for(int i = 0; i<counter; i++){
        fprintf(stderr,"Point at angle %f and distance %f \n",a[i],d[i]);

    }*/
    fprintf(stderr,"We detected %d points",counter);
    //std::ifstream file;
    //file.open("lidar_2112_v2.txt");
    //std::cout<<"c'est beacons qui marche pas?";
    //printf("counter=%d", counter);
    int obj_counter=0;
    std::vector<float> newa;
    std::vector<float> newd;
    //float newa[167]={};
    //float newd[167]={};
    float refd=d[0];
    float refa=a[0];
    float moyd=refd;
    float moya=refa;
    int obj_iter=0;
    int moy_count=1;
    double actualDistance;
    double objectMinWidth = 0.09;

    for (int i=1; i<counter;++i){
	//printf("dist: %f, angle %f \n", d[i],a[i]);
	//printf("dist: %f", distance(refa, a[i],refd, d[i]));
    actualDistance = distance(refa, a[i],refd, d[i]);
    //fprintf(stderr,"test distance %lf ", distance(1,180,1,1));
    //fprintf(stderr,"Distance between point (%f,%f) and (%f,%f) is %lf\n",refa,refd,a[i],d[i],actualDistance);
	if(actualDistance <= objectMinWidth){
	//if(d[i]-refd <=0.13){
	    //printf("a[i] %f \n", a[i]);
	    moyd+=d[i];
	    moya+=a[i];
	    //printf("moya %d \n", moya);
	    //printf("moyd %d \n", moyd);
	    moy_count+=1;
        refa=a[i];
	    refd=d[i];

	}
	else{
        fprintf(stderr,"object added with moya = %f\n",moya/float(moy_count));
        newa.push_back(moya/float(moy_count));
        newd.push_back(moyd/float(moy_count));
        
        refa=a[i];
        refd=d[i];
        moyd=refd;
        moya=refa;
        moy_count=1;
        obj_iter+=1;

	}
    if(i == counter-1){
        fprintf(stderr,"object added with moya = %f\n",moya/float(moy_count));
        newa.push_back(moya/float(moy_count));
        newd.push_back(moyd/float(moy_count));
        
        refa=a[i];
        refd=d[i];
        moyd=refd;
        moya=refa;
        moy_count=1;
        obj_iter+=1;
    }
    }
    for(int i = 0; i< newa.size();i++){
        fprintf(stderr,"Objet trouve à angle %f et distance %f \n",newa[i],newd[i]);
    }
    
    //printf("obj_iter=%d", obj_iter);
    /*
    for (int i=0; i<obj_iter;++i){
	    //printf(" i = %d and newa[i]: %lf \n",i, newa[i]); //chelou ce truc
	    //printf("newd[i]: %lf \n", newd[i]);
    }
    */
    
    //(float[2]) coord[3]={};
    int coord[3]={};
    std::vector<std::vector<float>> balises (3,std::vector<float>(2));
    for (int i = 0; i < obj_iter; i++) //Boucle pour trouver balises
    {
	//std::cout<<"il rentre dans la boucle?";
        float a1=newa[i];
        float d1=newd[i];
        for (int j = i+1; j<obj_iter; j++)//on va dans ce sens là pour aller plus vite
        {
	    //std::cout<<"et dans celle ci aussi?";
            float a2=newa[j];
            float d2=newd[j];
            for (int k = j+1; k<obj_iter; k++)
            {
                float a3=newa[k];
                float d3=newd[k];
                float triangle= distance(a1,a2,d1,d2)+distance(a1,a3,d1,d3)+distance(a2,a3,d2,d3); //sensé être 3+2.5+2.5 donc 8
                //printf("triangle: %f, i: %d, j: %d, k: %d \n", triangle, i, j, k);
		//printf("ai: %f, aj: %f, ak: %f \n", newa[i], newa[j], newa[k]);
		float dij=distance(newa[i],newa[j],newd[i],newd[j]);
		float djk=distance(newa[j],newa[k],newd[j],newd[k]);
		float dik=distance(newa[i],newa[k],newd[i],newd[k]);
		float triangleErrorTolerance = 0.15;//il est à 0.15 par défaut
		if(a1 < 22)        fprintf(stderr,"Nous avons un triangle de taille %f à angles %f %f %f \n",triangle,a1,a2,a3 );
		float isoceleTolerance = 0.2;
		
		if(triangle<=8.2 && triangle>=7.8 && dij<=3.3 && dij>=1.8 && djk<=3.3 && djk>=1.8 && dik<=3.3 && dik>=1.8 && (newd[i]+newd[j]<=6.6 && newd[k]+newd[j]<=6.6) && (newa[j]-newa[i])>=30.0 && (newa[k]-newa[j])>=30.0){//faudrait rajouter une condition brrr genre sur les anngles
		    //Ici c'est là où j'ai changé
		//if(triangle<=8+triangleErrorTolerance && triangle>=8-triangleErrorTolerance && dij<=3.2+triangleErrorTolerance && dij>=2-triangleErrorTolerance && djk<=3.2+triangleErrorTolerance && djk>=1.8+triangleErrorTolerance && dik<=3.3 && dik>=1.8+triangleErrorTolerance && (fabs(dij-djk)) < isoceleTolerance){//faudrait rajouter une condition brrr genre sur les anngles
		    fprintf(stderr,"On trouve un triangle \n");
		    coord[0]=i;//en théorie ce seront les bonnes
		    coord[1]=j;
		    coord[2]=k;
                    //std::cout<<"il trouve qqch";
		    
		    balises[0][0]=newa[coord[0]];
		    balises[0][1]=newd[coord[0]];
		    balises[1][0]=newa[coord[1]];
		    balises[1][1]=newd[coord[1]];
		    balises[2][0]=newa[coord[2]];
		    balises[2][1]=newd[coord[2]];
		    
		    printf("Balises: (%f,%f), (%f, %f), (%f, %f) \n",newa[coord[0]],newd[coord[0]], newa[coord[1]], newd[coord[1]], newa[coord[2]], newd[coord[2]] );
		    printf("triangle: %f \n", triangle);
		    float angle_b[3]={newa[coord[0]], newa[coord[1]], newa[coord[2]]};
		    float distance_b[3]={newd[coord[0]], newd[coord[1]], newd[coord[2]]};
		    float d01=distance(newa[coord[0]],newa[coord[1]], newd[coord[0]],newd[coord[1]]);
		    float d02=distance(newa[coord[0]],newa[coord[2]], newd[coord[0]],newd[coord[2]]);
		    float d12=distance(newa[coord[2]],newa[coord[1]], newd[coord[2]],newd[coord[1]]);
		    
		    float x3=2.0;
		    float y3= 0.0;
		    float x2=1.0;
		    float y2=3.0;
		    float x1=0.0;
		    float y1=0.0;

            //TEST AUGUSTIN CALCUL BALISES

                for (int i = 0; i < 3; i++) {
                    balises[i][0] = balises[i][0] * M_PI / 180.0;
                }
		//ici j'ai juste rajouté ces conditions là
                // Les coordonnées des balises
                double balise_coords[3][2] = {{0, 0}, {1, 3}, {2, 0}};
		    x3=2.0;
		    y3=0.0;
		    x2=1.0;
		    y2=3.0;
		    x1=0.0;
		    y1=0.0;
		
		if(d01 <=2.2 && d01>=1.7){
		    double balise_coords[3][2] = {{2, 0}, {0, 0}, {1, 3}};
		    x3=1.0;
		    y3=3.0;
		    x2=0.0;
		    y2=0.0;
		    x1=2.0;
		    y1=0.0;
		}
		else if(d02 <=2.2 && d02>=1.7){
		    double balise_coords[3][2] = {{0, 0}, {1, 3}, {2, 0}};
		    x3=2.0;
		    y3=0.0;
		    x2=1.0;
		    y2=3.0;
		    x1=0.0;
		    y1=0.0;
		}
		else if(d12 <=2.2 && d12>=1.7){
		    double balise_coords[3][2] = {{1, 3}, {2, 0}, {0, 0}};
		    x3=0.0;
		    y3=0.0;
		    x2=2.0;
		    y2=0.0;
		    x1=1.0;
		    y1=3.0;
		}
		
		

                // Calculer la position du robot
                double myX = 0, myY = 0;
                
		for (int i = 0; i < 3; i++) {
                    myX += balise_coords[i][0] + balises[i][1] * cos(balises[i][0]);
                    myY += balise_coords[i][1] + balises[i][1] * sin(balises[i][0]);
                }
		/*
		myX = balise_coords[2][0] + balises[2][1] * cos(balises[2][0]);
		myY = balise_coords[2][1] + balises[2][1] * sin(balises[2][0]);
		* */
                myX /= 3.0;
                myY /= 3.0;
                        printf("myX = %f myY = %f \n",myX,myY);

            //FIN TEST AUGUSTIN
		    
		    //now nex modified coordinates
		    float x1n = x1-x2;
		    float y1n = y1-y2;
		    float x3n = x3-x2;
		    float y3n = y3-y2;
		    //the three cot
		    float T12=1.0/(tan( newa[coord[1]]*(M_PI/180)-newa[coord[0]]*(M_PI/180)));
		    float T23=1.0/(tan(newa[coord[2]]*(M_PI/180)-newa[coord[1]]*(M_PI/180)));
		    float T31=(1.0-T12*T23)/(T12+T23);
		    //now the modified circle center coordinates
		    float x12n= x1n+T12*y1n;
		    float y12n= y1n-T12*x1n;
		    float x23n=x3n-T23*y3n;
		    float y23n=y3n+T23*x3n;
		    float x31n=(x3n+x1n)+T31*(y3n-y1n);
		    float y31n=(y3n+y1n)-T31*(x3n-x1n);
		    
		    float k31n=x1n*x3n+y1n*y3n+T31*(x1n*y3n-x3n*y1n);
		    float D=(x12n-x23n)*(y23n-y31n)-(y12n-y23n)*(x23n-x31n);
		    //printf("D: %f, x1: %f, x2: %f,, y2: %f x3: %f", D, x1, x2, y2, x3);
		    //printf("k31n: %f, y12n: %f, y23n: %f, x23n : %f, x12n: %f \n", k31n, y12n, y23n, x23n, x12n);
		    if (D==0.0){
			//std::cout<<"oula attention D est à 0";
		    }
		    float xr=x2+((k31n*(y12n-y23n))/D);
		    float yr=y2+((k31n*(x23n-x12n))/D);
		    printf("Alicia coords robots: xR= %f and Xy= %f \n", xr,yr);
		    myPos.x = myX;
		    myPos.y = myY;
		    //fprintf(stderr,"beacon data\n");
		    myPos.theta = angle_robot(balises);
		    return myPos;
		    
		    
		    //w_plot(&newa[0], &newd[0], angle_b, distance_b, obj_iter);
		    //detect_obstacle(newa, newd, obj_iter);
		    //return balises;
		    //break;//ici voir comment en sortir totalement
		    
                }
            }
            
        }
	
    }
    
    float d01=distance(newa[coord[0]],newa[coord[1]], newd[coord[0]],newd[coord[1]]);
    float d02=distance(newa[coord[0]],newa[coord[2]], newd[coord[0]],newd[coord[2]]);
    float d12=distance(newa[coord[2]],newa[coord[1]], newd[coord[2]],newd[coord[1]]);
    
    float x3=2.0;
    float y3= 0.0;
    float x2=1.0;
    float y2=3.0;
    float x1=0.0;
    float y1=0.0;
    
    //now nex modified coordinates
    float x1n = x1-x2;
    float y1n = y1-y2;
    float x3n = x3-x2;
    float y3n = y3-y2;
    //the three cot
    float T12=1.0/(tan( newa[coord[1]]*(M_PI/180)-newa[coord[0]]*(M_PI/180)));
    float T23=1.0/(tan(newa[coord[2]]*(M_PI/180)-newa[coord[1]]*(M_PI/180)));
    float T31=(1.0-T12*T23)/(T12+T23);
    //now the modified circle center coordinates
    float x12n= x1n+T12*y1n;
    float y12n= y1n-T12*x1n;
    float x23n=x3n-T23*y3n;
    float y23n=y3n+T23*x3n;
    float x31n=(x3n+x1n)+T31*(y3n-y1n);
    float y31n=(y3n+y1n)-T31*(x3n-x1n);
    
    float k31n=x1n*x3n+y1n*y3n+T31*(x1n*y3n-x3n*y1n);
    float D=(x12n-x23n)*(y23n-y31n)-(y12n-y23n)*(x23n-x31n);
    //printf("D: %f, x1: %f, x2: %f,, y2: %f x3: %f", D, x1, x2, y2, x3);
    //printf("k31n: %f, y12n: %f, y23n: %f, x23n : %f, x12n: %f \n", k31n, y12n, y23n, x23n, x12n);
    if (D==0.0){
	//std::cout<<"oula attention D est à 0";
    }
    float xr=x2+((k31n*(y12n-y23n))/D);
    float yr=y2+((k31n*(x23n-x12n))/D);
    //printf("coords robots: xR= %f and Xy= %f \n", xr,yr);
    myPos.x = 0.0;
    myPos.y = 0.0;
    //fprintf(stderr,"beacon data\n");
    return myPos;
}



int main(int argc, const char * argv[]){
    fprintf(stderr,"Argc  = %d\n",argc);
    int write_fd;
    if(argc > 1) write_fd = atoi(argv[1]); // Récupération du descripteur de fichier d'écriture du pipe à partir des arguments de la ligne de commande
    lidarPos position;
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
		lidar->startScanExpress(false, scanModes[0].id);
		//lidar->setMotorSpeed(0);

		LidarScanMode scanMode;//on utilise le mode standard de scan(on peut aussi choisir)
		lidar->startScan(false, true, 0, &scanMode);

		sl_lidar_response_measurement_node_hq_t nodes[8192];//on définit un format de réponse
		size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);//on définit la taille du truc
        while(1){
		sl_result res_gscan = lidar->grabScanDataHq(nodes, nodeCount);//on remplit avec le grab data (ici hq pas nécéssaire, <16m)
		//res_gscan_int= lidar->grabScanDataWithInterval(nodes, nodeCount);//faudrait checker la diff avec le continu
		if (IS_OK(res_gscan)){
		    //fprintf(stderr, "Hey mais... le grabscan marche");//erreur si je sais pas grab les data
		    lidar->ascendScanData(nodes, nodeCount);
		    //std::ofstream out("lidar_bord_g_vers2.txt");
		    float angle[nodeCount]={};
            //fprintf(stderr,"Check 1\n");
		    float distance[nodeCount]={};
		    int counter=0;
		    for(int i=0;i<(int)nodeCount;i++){
                //fprintf(stderr,"Check 2\n");
			float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
			float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
			//out << angle_in_degrees << " , " << distance_in_meters << "\n";
                //fprintf(stderr,"Check 3\n");
            float limit_of_detection = 3.6;
			if(distance_in_meters<=limit_of_detection && distance_in_meters!=0.0){
			    angle[counter]=angle_in_degrees;
			    distance[counter]=distance_in_meters;
			    counter+=1;
			    //out << angle_in_degrees << " , " << distance_in_meters << "\n";
			    //printf("Angle : %f, Distance : %f \n", angle_in_degrees,distance_in_meters);
			}
			
		    }
            //fprintf(stderr,"Check 4\n");
		    position = beacon_data(angle, distance, counter);
            //fprintf(stderr,"Check 5\n");
		    //std::vector<std::vector<float>> balises= beacon_data(angle, distance, counter);
		    //angle_robot(balises);
		    //out << balises[0][0] << "," << balises[0][1] << "||" << balises[1][0] << "," << balises[1][1] << "||" <<balises[2][0]<< "," << balises[2][1]<<"\n";
		    //printf("les balises sont en: b1(%f, %f), b2(%f, %f), b3(%f,%f)", balises[0][0], balises[0][1], balises[1][0], balises[1][1], balises[2][0], balises[2][1]);
		    
		    //out.close();
		    //std::cout<<"alors tu arrives jusqu'ici ou pas?";
		    //plot_histogram(nodes, nodeCount);
		}
		else{
			fprintf(stderr, "OSKUR poupon, failed to grab scan the data with LIDAR %08x\r\n", res_gscan);//erreur si je sais pas grab les data
		
		    //ici faut recup les donner de res scan
		}
	    
	    //fin de la bouboucle
	    //std::cout<<"fin de programme, arrête toi sale bête";
        //fprintf(stderr,"Check 6\n");
        if(argc >1){
        //fprintf(stderr,"Check 6,5\n");
        
        std::vector<float> numbers = {position.x,position.y,position.theta}; // Déclaration du tableau de nombres à virgule flottante à envoyer
        //fprintf(stderr,"Check 7\n");
        write(write_fd, numbers.data(), numbers.size() * sizeof(float)); // Écriture des nombres dans le pipe
        }
        fprintf(stderr,"position x = %f \n",position.x);
        fprintf(stderr,"position y = %f \n",position.y);
        fprintf(stderr,"position theta = %f \n",position.theta);
        //fprintf(stderr,"Check 8\n");
        sleep(5);

        }
	//sleep(1);
        }else{
            fprintf(stderr, "OSKUR poupon, failed to get device information from LIDAR %08x\r\n", res);
        }
	
    
	    //sleep(5);
	    //lidar->setMotorSpeed(1);
    
    }else{
        fprintf(stderr, "OSKUR poupon, Failed to connect to LIDAR %08x\r\n", res);
    }
    //clock_t end= clock();
    //double time_spent= (double)(end-begin)/CLOCKS_PER_SEC;
    //printf("execution time: %f \n", time_spent);
    //close(write_fd); // Fermeture du descripteur de fichier d'écriture du pipe
}
