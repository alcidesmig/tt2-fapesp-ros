#include <stdio.h>
#include <stdlib.h>
#include <string.h>


void filterData(char *source, char *dest)
{
    FILE *data = fopen(source, "r");
    FILE *filtered_data = fopen(dest, "w");
    char line[256];
    while(fscanf(data, "%s", line) > 0)
    {
    	printf("%s\n", line);
    	char * token = strtok(line, ";");
        for(int i = 0; i < 9; i ++)
        {
            token = strtok(NULL, ";");
            double value;
            printf("%s\n", token);
            fprintf(filtered_data, "%s;", token);
        }
        fprintf(filtered_data, "\n");
    }
}

int main()
{
	filterData("data.txt", "filtered_data.txt");
    return 0;
}