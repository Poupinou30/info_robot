//ici c'est cam.c
//#include "cam.cpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define MAX_LINE_LENGTH 256

int main(){
    /*
    int code= system(cam.cpp);

    // checking if the command was executed successfully
    if (code == 0) {
        cout << "Command executed successfully, c'est good pouponnn" << endl;
    }
    else {
        cout << "Command execution failed or returned "
                "non-zero: "
             << returnCode << endl;
    }
    * */
    FILE *fp;
    char path[MAX_LINE_LENGTH];
    //char *token;
    //int x, y, z;
    float coordinates[6][3];
    int num_sublists = 0;
    fp = popen("python3 agUrco.py", "r");//on ouvre le fichier python
    if(fp == NULL){
        printf("Attention le code python run pas \n");
        return 1;
    }
    
    
    while(fgets(path, sizeof(path), fp) != NULL){//pourquoi il enlève le -1 maintenant?
        if(path[0] =='['){
            int count=0;
            char *token = strtok(path, "[], \n");
            printf("token: %s\n, %d", token);
            while(token!=NULL && count< 3){
                errno = 0;
                coordinates[num_sublists][count] = strtof(token, NULL);
                if(coordinates[num_sublists][count] == 0.0000 && errno != 0){
                    perror("strtof");
                }
                token = strtok(NULL, "[], \n");
                count++;
            }
            if(count== 3){
                num_sublists++;
            }
            
            printf("%d", num_sublists);
            if(num_sublists >= 6){
                printf("chelou ça");
                break;
            } 
        }
        
   }
   printf("%d", num_sublists);
   for(int i = 0; i < num_sublists;i++){
        printf("Sublist %d : (%.6f, %.6f, %.6f)\n", i+1, coordinates[i][0], coordinates[i][1], coordinates[i][2]);
   }     
        /*
        if(sscanf(path, "%f %f %f", &x, &y, &z)==3){
            printf("Received coord: (%.6f, %.6f, %.6f)\n", x, y, z);
        }else {
            printf("Error: Failed to parse coord from input.\n");
        }
        
        token = strtok(path, "\n");
        printf("token: %s\n", token);
        float coord_list[3]={};
        int counter=0;
        //printf("token: %s\n, %d", token);
        
        while (token!=NULL && counter <3){
            printf("ici counter %d", counter);
            errno = 0;
            if(counter ==0){
                x=strtof(token, NULL);
                printf("%f", x);
            }
            else if(counter ==1){
                y=strtof(token, NULL);
            }
            else if(counter ==2){
                z=strtof(token, NULL);
            }
            if(errno !=0){
                perror("strtof");
            }
            
            token = strtok(NULL, " ");
            
            //float float_val= strtof(token, NULL);
            //printf("Received float value : %.6f\n", float_val);
            //coord_list[counter]=float_val;
            counter+=1;
            * 
        */   
        
        //printf("Received coord: (%.6f, %.6f, %.6f)\n", x, y, z);
        //printf("%d \n", counter);
        //printf("la liste: %f, %f, %f \n", coord_list[0], coord_list[1], coord_list[2]);
        
        /*
        //int x, y, z;
        if(sscanf(path, "%f %f %f", &x, &y, &z)==3){
            printf("Received coordinates : (%f, %f, %f)\n", x, y, z);
            //coord_list[0]=x;
            //coord_list[1]=y;
            //coord_list[2]=z;
        */
        
    
    
    pclose(fp);
    return 0;
}
