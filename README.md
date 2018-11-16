# squirrel_prediction repository


Repository for prediction and learning related SQUIRREL packages.

Installation requiremets
```
rosdep install --from-path squirrel_prediction -i -y
```

Running service for predictions
==================================
After installation of the repository you can run the service for predictions as:
```
rosrun squirrel_relations_prediction predict_relations_server.py 
```

Service call for relations prediction:
===============

```
# input parameters
string data_path
string input_file
string output_file
int32 number_of_columns
---
#output
bool finished
```
Input file should contain the representation of a graph with missing values.  
First two columns have to contain string IDs of origin adn destination vertices. Rest columns represent the edges. The file has to be in .csv format where columns are separated with commas and rows with a new line. Empty values mark unknown edges in a graph. '0' marks no edge between vertices and '1' marks existance of the dge between two vertices.
