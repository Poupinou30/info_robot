//ici c'est cam.c
//#include "cam.cpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define MAX_LINE_LENGTH 256


int main(){
    FILE *fp;
    char  path[256];
    char num_plants[100];
    fp = popen("python3 Aruco.py", "r");
    if(fp==NULL){
        printf("Attention le code python run pas \n");
        return 1;
    }    
    if(fgets(path, sizeof(path), fp) != NULL){
        printf("Donnee venant du fichier python : %s \n", path);
        strcpy(num_plants, path);
    }
    pclose(fp);
    printf("Donnee venant du fichier python : %s \n", num_plants);
    return 0;
}

int main2(){
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
    fp = popen("python3 Aruco.py", "r");//on ouvre le fichier python
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
    pclose(fp);
    return 0;
}
