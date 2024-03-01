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

typedef struct positionBot{
    float x;
    float y;
    float theta;
} positionBot;


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
float distance (float a1,float a2,float d1,float d2){
    float dist= sqrt(pow(d1,2)+pow(d2,2) - (2*d1*d2*cos((a1-a2)*M_PI/180)));
    return dist;
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

//Fonction calculs
std::vector<std::vector<float>> beacon_data(float a[] ,float d[],int counter,positionBot* position ){
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
    for (int i=1; i<counter;++i){
	//printf("dist: %f, angle %f \n", d[i],a[i]);
	//printf("dist: %f", distance(refa, a[i],refd, d[i]));
	if(distance(refa, a[i],refd, d[i])<=0.09 ){
	//if(d[i]-refd <=0.13){
	    //printf("a[i] %f \n", a[i]);
	    moyd+=d[i];
	    moya+=a[i];
	    //printf("moya %d \n", moya);
	    //printf("moyd %d \n", moyd);
	    moy_count+=1;
	}
	else{
        newa.push_back(moya/float(moy_count));
        newd.push_back(moyd/float(moy_count));
	    //newa[obj_iter]=moya/float(moy_count);
	    //newd[obj_iter]=moyd/float(moy_count);
	    //printf("i = %d and newa : %f, newd: %f \n",obj_iter,newa[obj_iter],newd[obj_iter]);
	    //printf("newd : %f \n",newd[obj_iter]);
	    
	    refa=a[i];
	    refd=d[i];
	    moyd=refd;
	    moya=refa;
	    moy_count=1;
	    obj_iter+=1;
	}
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
    for (int i = 0; i < obj_iter; i++)
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
		
		if(triangle<=8.0 && triangle>=7.89 && dij<=3.3 && dij>=1.9 && djk<=3.3 && djk>=1.9 && dik<=3.3 && dik>=1.9 && (newd[i]+newd[j]<=3.5 && newd[k]+newd[j]<=3.5) && (newa[j]-newa[i])>=30.0 && (newa[k]-newa[j])>=30.0){//faudrait rajouter une condition brrr genre sur les anngles
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
		    if(verbose){
                printf("Balises: (%f,%f), (%f, %f), (%f, %f) \n",newa[coord[0]],newd[coord[0]], newa[coord[1]], newd[coord[1]], newa[coord[2]], newd[coord[2]] );
                printf("triangle: %f \n", triangle);}

		    float angle_b[3]={newa[coord[0]], newa[coord[1]], newa[coord[2]]};
		    float distance_b[3]={newd[coord[0]], newd[coord[1]], newd[coord[2]]};
		    //w_plot(&newa[0], &newd[0], angle_b, distance_b, obj_iter);
		    //detect_obstacle(newa, newd, obj_iter);
		    //return balises;
		    //break;//ici voir comment en sortir totalement
		    
                }
            }
            
        }
	
    }
    
    //printf("a1: %f, a2: %f, a3: %f \n",newa[coord[0]], newa[coord[1]], newa[coord[2]] );
    //maintenant partie calculs des coords
    /*
    on va d'abord checker quelles sont les balises sur le bord, puis en poser une en 0,0, l'autre en 0,3, l'autre en 2,1.5 et le robot au milieu de ses coords là
    */
    //printf("balise 1: dist1: %f, angle1: %f - balise 2: dist2: %f, angle2: %f - balise 3: dist3: %f, angle3: %f",newd[coord[0]], newa[coord[0]], newd[coord[1]], newa[coord[1]], newd[coord[2]], newa[coord[2]] ); 
    //printf("test x1:%f, y1: %f", newd[coord[0]]*cosf( -newa[coord[0]]*(M_PI/180)), newd[coord[0]]*sinf( -newa[coord[0]]*(M_PI/180)));
    //printf("test x2:%f, y2: %f", newd[coord[1]]*cosf( -newa[coord[1]]*(M_PI/180)), newd[coord[1]]*sinf( -newa[coord[1]]*(M_PI/180)));
    //printf("test x3:%f, y3: %f", newd[coord[2]]*cosf( -newa[coord[2]]*(M_PI/180)), newd[coord[2]]*sinf( -newa[coord[2]]*(M_PI/180)));


    /*float x1=newd[coord[0]]*cosf( -newa[coord[0]]*(M_PI/180));
    float y1=newd[coord[0]]*sinf( -newa[coord[0]]*(M_PI/180));
    float x2=newd[coord[1]]*cosf( -newa[coord[1]]*(M_PI/180));
    float y2=newd[coord[1]]*sinf( -newa[coord[1]]*(M_PI/180));
    float x3=newd[coord[2]]*cosf( -newa[coord[2]]*(M_PI/180));
    float y3=newd[coord[2]]*sinf( -newa[coord[2]]*(M_PI/180));
    */
    
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
	std::cout<<"oula attention D est à 0";
    }
    float xr=x2+((k31n*(y12n-y23n))/D);
    float yr=y2+((k31n*(x23n-x12n))/D);
    float theta = 0;
    position->x = xr;
    position->y = yr;
    position->theta = theta;

    if(verbose) printf("coords robots: xR= %f and Xy= %f \n", xr,yr);
    
    return balises;
}
/*

void plot_histogram(sl_lidar_response_measurement_node_hq_t * nodes, size_t count)//ça vient de la lib sdk
{
    const int BARCOUNT =  75;
    const int MAXBARHEIGHT = 20;
    // const float ANGLESCALE = 360.0f/BARCOUNT;

    float histogram[BARCOUNT];
    for (int pos = 0; pos < _countof(histogram); ++pos) {
        histogram[pos] = 0.0f;
    }

    float max_val = 0;
    for (int pos =0 ; pos < (int)count; ++pos) {
        int int_deg = (int)(nodes[pos].angle_z_q14 * 90.f / 16384.f);
        if (int_deg >= BARCOUNT) int_deg = 0;
        float cachedd = histogram[int_deg];
        if (cachedd == 0.0f ) {
            cachedd = nodes[pos].dist_mm_q2/4.0f;
        } else {
            cachedd = (nodes[pos].dist_mm_q2/4.0f + cachedd)/2.0f;
        }

        if (cachedd > max_val) max_val = cachedd;
        histogram[int_deg] = cachedd;
    }

    for (int height = 0; height < MAXBARHEIGHT; ++height) {
        float threshold_h = (MAXBARHEIGHT - height - 1) * (max_val/MAXBARHEIGHT);
        for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
            if (histogram[xpos] >= threshold_h) {
                putc('*', stdout);
            }else {
                putc(' ', stdout);
            }
        }
        printf("\n");
    }
    for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
        putc('-', stdout);
    }
    printf("\n");
}
*/

int main(int argc, const char * argv[]){
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
		std::ifstream file;
		file.open("lidar_bord_g_vers2.txt");
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

        while(1) {

            sl_result res_gscan = lidar->grabScanDataHq(nodes,
                                                        nodeCount);//on remplit avec le grab data (ici hq pas nécéssaire, <16m)
            //res_gscan_int= lidar->grabScanDataWithInterval(nodes, nodeCount);//faudrait checker la diff avec le continu
            if (IS_OK(res_gscan)) {
                if(verbose) fprintf(stderr, "Hey mais... le grabscan marche");//erreur si je sais pas grab les data
                lidar->ascendScanData(nodes, nodeCount);
                std::ofstream out("lidar_bord_g_vers2.txt");
                float angle[nodeCount] = {};
                float distance[nodeCount] = {};
                int counter = 0;
                for (int i = 0; i < (int) nodeCount; i++) {

                    float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
                    float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
                    //out << angle_in_degrees << " , " << distance_in_meters << "\n";

                    if (distance_in_meters <= 3.6 && distance_in_meters != 0.0) {
                        angle[counter] = angle_in_degrees;
                        distance[counter] = distance_in_meters;
                        counter += 1;
                        out << angle_in_degrees << " , " << distance_in_meters << "\n";
                        //printf("Angle : %f, Distance : %f \n", angle_in_degrees,distance_in_meters);
                    }

                }

                //beacon_data(angle, distance, counter);
                positionBot *position = (positionBot*) malloc(sizeof(positionBot));
                std::vector <std::vector<float>> balises = beacon_data(angle, distance, counter,position);
                out << balises[0][0] << "," << balises[0][1] << "||" << balises[1][0] << "," << balises[1][1] << "||"
                    << balises[2][0] << "," << balises[2][1] << "\n";
                //printf("les balises sont en: b1(%f, %f), b2(%f, %f), b3(%f,%f)", balises[0][0], balises[0][1], balises[1][0], balises[1][1], balises[2][0], balises[2][1]);

                out.close();
                //std::cout<<"alors tu arrives jusqu'ici ou pas?";
                //plot_histogram(nodes, nodeCount);
            } else {
                fprintf(stderr, "OSKUR poupon, failed to grab scan the data with LIDAR %08x\r\n",
                        res_gscan);//erreur si je sais pas grab les data

                //ici faut recup les donner de res scan
            }

            //fin de la bouboucle
            std::cout << "fin de programme, arrête toi sale bête";
        }
	    
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
}



