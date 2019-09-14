import trajectory as track


class Indicators:
    @staticmethod
    def mean_square_distance(trajectory):
        sum_distance = 0
        sum_time_interval = 0
        start_point = trajectory.point_list[0]
        for i in range(len(trajectory.point_list) - 1):
            integral_start_point = trajectory.point_list[i]
            integral_end_point = trajectory.point_list[i + 1]
            if trajectory.coordinate == 'geo':
                integral_start_dist = track.distance_spatial(start_point.x, start_point.y, integral_start_point.x,
                                                             integral_start_point.y)
                integral_end_dist = track.distance_spatial(start_point.x, start_point.y, integral_end_point.x,
                                                           integral_end_point.y)
            else:
                integral_start_dist = track.distance(start_point.x, start_point.y, integral_start_point.x,
                                                     integral_start_point.y)
                integral_end_dist = track.distance(start_point.x, start_point.y, integral_end_point.x,
                                                   integral_end_point.y)
            integral_start_time = integral_start_point.semantic_info[1]
            integral_end_time = integral_end_point.semantic_info[1]
            sum_distance += (0.5 * (integral_end_dist + integral_start_dist) *
                             (integral_end_time - integral_start_time))
            sum_time_interval += (integral_end_time - integral_start_time)
        return sum_distance / sum_time_interval

    @staticmethod
    def probability_distance_function():
        return 0

    @staticmethod
    def probability_density_function():
        return 0
