import json

data_file = "/home/noelan/chalmers/kandidatarbete/Autonomus-Lawnmower/tests/data/f9_m8t_1128/f9_post_2346.txt"

def data_extraction(TEXTFILE):
    with open(TEXTFILE, 'r', encoding='utf-8') as f:
        contents = f.read().split('\n')
        
    for i in range(len(contents)):
        contents[i] = contents[i].split()
    del contents[0][0]
    f.close()
    
    return contents

data_list = data_extraction(data_file)


def build_data(data):
    data_dict_1 = {}
    data_dict_2 = {}
    time_list_1 = []
    time_list_2 = []
    del data[0]

    for line in data:
        if line[0] == "2018/11/28":   
            data_dict_1[line[1]] =    {
                                    "lat": float(line[2]),
                                    "lon": float(line[3])
                                    }
            time_list_1.append(line[1])
        if line[0] == "2018/11/29":   
            data_dict_2[line[1]] =    {
                                    "lat": float(line[2]),
                                    "lon": float(line[3])
                                    }
            time_list_2.append(line[1])
        data_dict_1["Time"] = time_list_1
        data_dict_2["time"] = time_list_2


        
    json_object1 = json.dumps(data_dict_1, indent=2, ensure_ascii=True)
    json_object2 = json.dumps(data_dict_2, indent=2, ensure_ascii=True)
 
    with open("tests/gps_2018-11-28_new.json", "w",) as outfile:
        outfile.write(json_object1)

    with open("tests/gps_2018-11-29_new.json", "w",) as outfile:
        outfile.write(json_object2)

build_data(data_list)