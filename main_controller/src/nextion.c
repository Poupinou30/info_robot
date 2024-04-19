#include "headers.h"

int UART_handle_nextion;
float tempoMyX, tempoMyY, tempoOppX, tempoOppY;
char myScore[25];

int initializeUART(){
    int baud_rate = 9600;
    
    UART_handle = serOpen("/dev/ttyAMA0",baud_rate,0); //ttyS0 est le port UART, 0 est le flag du mode à utiliser
    if (UART_handle < 0){
        fprintf(stderr, "Erreur lors de l'ouverture de la connexion UART.\n");
        exit(1);    // Exit the program in case of error
    }

    struct termios tty;
    if(tcgetattr(UART_handle, &tty) != 0) {
        perror("Failed to get attributes of the serial port");
        return 1;
    }

    // Configuring Port Settings for 9600 Baud
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB; // No Parity
    tty.c_cflag &= ~CSTOPB; // 1 Stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    tty.c_lflag |= ICANON;
    tty.c_lflag |= ECHO; // Disable echo
    tty.c_lflag |= ECHOE; // Disable erasure
    tty.c_lflag |= ECHONL; // Disable new-line echo
    tty.c_lflag |= ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    if (tcsetattr(UART_handle, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return 1;
    }

    return UART_handle;
}

uint8_t UART_send(int UART_handle, char* data){
    if (VERBOSE) printf("Sending '%s' by UART\n", data);
    if(serWrite(UART_handle, data, strlen(data))!=0){
        fprintf(stderr,"Error while writing to UART\n");
        return 0;
    }
    //int bytes_written = serWrite(UART_handle, data, strlen(data));
    //if (bytes_written != strlen(data)) {
    //    fprintf(stderr, "Error writing to UART.\n");
    //    return 0;
    //}
    else if (VERBOSE) printf("UART data sent successfully\n");
    return 1;
}

uint8_t UART_send_commands(int UART_handle, char* data){
    unsigned char endString = 0xff;
    if (UART_send(UART_handle, data) != 1) return 0;
    serWrite(UART_handle, &endString,1);
    serWrite(UART_handle, &endString,1);
    serWrite(UART_handle, &endString,1);
    return 1;
}

uint8_t UART_receive(int UART_handle, char* received){
    char tempoChar[100] = "";
    
    int bytesRead = serRead(UART_handle, tempoChar, 255);
    if (bytesRead < 0) {
        fprintf(stderr, "Error reading from UART.\n");
        return 0; // Return 0 to indicate failure
    } else if (bytesRead > 0) {
        strcat(received,tempoChar);
        //strcpy(received, tempoChar);
        if(tempoChar[bytesRead-1] == '>') {
            tempoChar[bytesRead] = '\0'; // Null-terminate the string
            return 1;
        }
    }
    return 0;
}

void handleCommand(char *string) {

    char identifier[50];  // Buffer for the identifier
    int value;            // Integer to store the extracted number

    if (strcmp(receivedChars, "<start>") == 0){
        go = 1;
        enqueue(q, "tm0.en=1");
        nextionStart = 1;
        printf("Go go gooo\n");
    }
    else if (strcmp(receivedChars, "<back>") == 0){
        go = 0;
        printf("back\n");
    }else if ((strcmp(receivedChars, "<stop>") == 0)){
        go = 0;
        printf("stop\n");
    }else if (strcmp(receivedChars, "<finish>") == 0){
        finish = true;
        printf("finish\n");
    }
    else if (strcmp(receivedChars, "<blue>") == 0 || strcmp(receivedChars, "<yellow>") == 0) {
        strcpy(myTeam, receivedChars); // Copie de la valeur de tempoChar dans myTeam
        printf("Received team: %s\n", receivedChars);
        UART_send_commands(UART_handle, "color.en=1");
    }
    else if (strcmp(receivedChars, "<home>") == 0){
        strcpy(myPage, receivedChars); // Copie de la valeur de tempoChar dans myTeam
        printf("myPage: %s\n", receivedChars);
        UART_send_commands(UART_handle, "pageHome.en=1");
    }
    else if (sscanf(receivedChars, "<%s = %d>", identifier, &value) == 2) {
        if (strcmp(identifier, "startY") == 0) {
            startY = value;
            printf("startY: %d\n", startY);
            UART_send_commands(UART_handle, "posY.en=1");
        }else if (strcmp(identifier, "startB") == 0) {
            startB = value;
            printf("startB: %d\n", startB);
            UART_send_commands(UART_handle, "posB.en=1");
        }
    }
}

Queue* createQueue() {
    Queue* q = (Queue*) malloc(sizeof(Queue));
    q->front = q->rear = NULL;
    return q;
}

void enqueue(Queue* q, const char* value) {
    Node* newNode = (Node*) malloc(sizeof(Node));
    newNode->data = strdup(value);  // Allocate memory and copy string
    newNode->next = NULL;
    
    if (q->rear == NULL) {
        q->front = q->rear = newNode;
        return;
    }
    
    q->rear->next = newNode;
    q->rear = newNode;
}

char* dequeue(Queue* q) {
    if (q->front == NULL) {
        return NULL; // Indicate queue is empty
    }
    
    Node* temp = q->front;
    char* data = temp->data;
    q->front = q->front->next;
    
    if (q->front == NULL) {
        q->rear = NULL;
    }
    
    free(temp);
    return data;
}

void display(Queue* q) {
    Node* temp = q->front;
    printf ("Queue : ");
    while (temp != NULL) {
        printf("%s ", temp->data);
        temp = temp->next;
    }
    printf("\n");
}

void map_coordinates(float x_meters, double y_meters, char* x_map, char* y_map, int player) {
    // Original map dimensions in meters
    double original_width_m = 2.0;   // meters
    double original_height_m = 3.0;  // meters

    // New map dimensions in millimeters (now flipped for x and y)
    double new_width_mm = 483.0;     // millimeters (originally y dimension)
    double new_height_mm = 322.0;    // millimeters (originally x dimension)

    // Convert new map dimensions to meters for scaling calculation
    double new_width_m = new_width_mm / 1000.0;  // meters
    double new_height_m = new_height_mm / 1000.0; // meters

    // Calculate scaling factors
    double scale_x = new_height_m / original_width_m;  // Flip scale to match dimensions
    double scale_y = new_width_m / original_height_m;

    // Apply the scaling to input coordinates
    double x_scaled = x_meters * scale_x;
    double y_scaled = y_meters * scale_y;

    // Apply the origin offset of the new map (convert it to meters)
    double origin_offset_x_m = 121.0 / 1000.0;  // meters (originally y offset)
    double origin_offset_y_m = 47.0 / 1000.0;   // meters (originally x offset)

    // Calculate the final coordinates in millimeters
    // Inverted x and y for final map coordinates
    // Adjust for square centering by subtracting half of the square's dimensions
    double y_mm = (x_scaled + origin_offset_x_m) * 1000.0 - 29.0;  // 29 mm is half the width of the square
    double x_mm = (y_scaled + origin_offset_y_m) * 1000.0 - 29.0;  // 29 mm is half the height of the square

    // Assuming x_map and y_map point to buffers large enough to hold the resulting strings
    if (player == 0) {
        sprintf(x_map, "posBB.x=%d", (int)x_mm); // Cast to int for printing
        sprintf(y_map, "posBB.y=%d", (int)y_mm);
    } else {
        sprintf(x_map, "posOpp.x=%d", (int)x_mm);
        sprintf(y_map, "posOpp.y=%d", (int)y_mm);
    } 
}

// quand on appelle cette fonction toute les 30ms il faudra aussi qu'on ait bien la valeur now
void nextion_communication(){

    // Condition obligatoire avant de lancer toute communication
    if(nowValue - (startInitialization.tv_sec*1000+startInitialization.tv_usec/1000) > 3000){
        //On l'envoit slmt au début après ca sert plus à rien
        if(nowValue - (startInitialization.tv_sec*1000+startInitialization.tv_usec/1000) < 3050){
            char* isReady = "Rasp.en=1";
            UART_send_commands(UART_handle, isReady);
        }

        // Enqueue the data every 350ms
        if(nowValue - (endQueue.tv_sec*1000+endQueue.tv_usec/1000) > 350){
            
            // My position
            pthread_mutex_lock(&lockFilteredPosition);
            tempoMyX = *myFilteredPos.x;
            tempoMyY = *myFilteredPos.y;
            pthread_mutex_unlock(&lockFilteredPosition);
            char x_map_bb[25]; 
            char y_map_bb[25];
            map_coordinates(tempoMyX, tempoMyY, x_map_bb, y_map_bb, 0);
            enqueue(q, x_map_bb);
            enqueue(q, y_map_bb);

            //Position of the opponent
            pthread_mutex_lock(&lockFilteredOpponent);
            tempoOppX = *myFilteredOpponent.x;
            tempoOppY = *myFilteredOpponent.y;
            pthread_mutex_unlock(&lockFilteredOpponent);
            char x_map_opp[25]; 
            char y_map_opp[25];
            map_coordinates(tempoOppX, tempoOppY, x_map_opp, y_map_opp, 1);
            enqueue(q, x_map_opp);
            enqueue(q, y_map_opp);

            // My score
            sprintf(myScore, "score.val=%d", score);
            enqueue(q, myScore);
            
            gettimeofday(&endQueue, NULL);
            endValueQueue = endQueue.tv_sec*1000+endQueue.tv_usec/1000;
        }
        
        if(go == 1){
            while(q->front != NULL){
                display(q);
                char* str = dequeue(q);
                if (UART_send_commands(UART_handle, str) != 1){
                    enqueue(q, str);
                }
            }
        }else if (go == 0){
            enqueue(q, "score.val=0");
            enqueue(q, "tm0.en=0");
            enqueue(q, "timer.val=0");
            enqueue(q, "fill 21,26,133,57,10565");
            go = 2;
            while(q->front != NULL){
                display(q);
                char* str = dequeue(q);
                if (UART_send_commands(UART_handle, str) != 1){
                    enqueue(q, str);
                }
            }
        }
        if (UART_receive(UART_handle, receivedChars)){
            printf("received char : %s\n", receivedChars);
            handleCommand(receivedChars);
            receivedChars[0] = '\0';
        }
    }
}