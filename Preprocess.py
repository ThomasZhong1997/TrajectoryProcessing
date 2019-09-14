import pandas as pd
import numpy as np
from osgeo import ogr
import trajectory as track


class Preprocess:
    @staticmethod
    def combine_base_station_record(record_filepath, station_filepath, connection_column, output_filepath):
        record_dataset = pd.read_csv(record_filepath, sep=',', header=0)
        record_list = []
        record_fields = record_dataset.columns.values.tolist()
        for index, one_record in record_dataset.iterrows():
            new_record = {}
            for field in record_fields:
                new_record[field] = one_record[field]
            record_list.append(new_record)
        station_list = {}
        station_dataset = pd.read_csv(station_filepath, sep=',', header=0)
        station_fields = station_dataset.columns.values.tolist()
        station_fields.remove(connection_column)
        for index, one_station in station_dataset.iterrows():
            index_string = one_station[connection_column]
            inner_content = []
            for field in station_fields:
                inner_content.append(one_station[field])
            station_list[index_string] = inner_content
        for field in station_fields:
            record_fields.append(field)
        for record in record_list:
            station_id = record[connection_column]
            connection_station = station_list[station_id]
            for i in range(len(connection_station)):
                record[station_fields[i]] = connection_station[i]
        combine_stream = open(output_filepath, 'w')
        sample_record = record_list[0]
        key_string = ''
        for key in sample_record:
            key_string += str(key)
            key_string += ','
        key_string = key_string[:-1]
        combine_stream.write(key_string + '\n')
        for record in record_list:
            record_string = ''
            for attribute in record:
                record_string += str(record[attribute])
                record_string += ','
            record_string = record_string[:-1]
            combine_stream.write(record_string + '\n')
        combine_stream.close()

    @staticmethod
    def calculate_boundary(trajectory_set):
        x_list = []
        y_list = []
        for trajectory in trajectory_set:
            for one_point in trajectory:
                x_list.append(one_point.x)
                y_list.append(one_point.y)
        result = (min(x_list), max(x_list), max(y_list), min(y_list))
        return result

    @staticmethod
    def map_matching(trajectory_set, vector_road_filepath, spatial_index='grid', index_scale=10, function_type='common'):
        set_boundary = Preprocess.calculate_boundary(trajectory_set)
        top_left_x = set_boundary[0]
        bottom_right_x = set_boundary[1]
        top_left_y = set_boundary[2]
        bottom_right_y = set_boundary[3]
        if trajectory_set.coordinate == 'geo':
            a = 1
        spatial_index = np.zeros([1, 1], np.int)
        if function_type == 'common':
            for trajectory in trajectory_set:
                if trajectory.coordinate == 'geo':
                    a = 1
        return 0


Preprocess.combine_base_station_record('XGrfid_day.csv', 'XGReaderId.csv', 'ReaderId', 'combine.csv')
