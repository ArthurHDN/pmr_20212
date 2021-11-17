import numpy as np

class PotentialFields():
    
    @staticmethod
    def calculate_attractive_vector(q,q_goal,saturate_distance,attractive_gain):
        pf = np.linalg.norm(q - q_goal)
        if pf <= saturate_distance:
            Fatt = -attractive_gain*(q-q_goal)
        else:
            Fatt = -(saturate_distance*attractive_gain/pf)*(q-q_goal)
        return Fatt

    @staticmethod
    def calculate_repulsive_vector(q, nearest_obstacle_position, obstacle_maximum_effect_distance, repulsive_gain):
        p = np.linalg.norm(q-nearest_obstacle_position)
        if p <= obstacle_maximum_effect_distance:
            Frep = (repulsive_gain*(1/p - 1/obstacle_maximum_effect_distance)/(p**2))*(q - nearest_obstacle_position)/p
        else:
            Frep = np.array((0,0,0))
        return Frep

    @staticmethod
    def calculate_vector(q,q_goal,nearest_obstacle_position,saturate_distance,attractive_gain,obstacle_maximum_effect_distance, repulsive_gain):
        Fatt = PotentialFields.calculate_attractive_vector(q, q_goal, saturate_distance, attractive_gain)
        Frep = PotentialFields.calculate_repulsive_vector(q,nearest_obstacle_position,obstacle_maximum_effect_distance,repulsive_gain)
        return Fatt + Frep