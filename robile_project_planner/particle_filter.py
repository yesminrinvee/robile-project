import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import math


class ParticleFilter:
    def __init__(self, num_particles=1000):
        self.num_particles = num_particles
        self.particles = []
        self.weights = []
        self.map = None
        
    def initialize_particles(self, initial_pose):
        """Initialize particles around initial pose"""
        self.particles = []
        for _ in range(self.num_particles):
            # Add Gaussian noise to initial pose
            x = initial_pose[0] + np.random.normal(0, 0.5)
            y = initial_pose[1] + np.random.normal(0, 0.5)
            theta = initial_pose[2] + np.random.normal(0, 0.1)
            self.particles.append([x, y, theta])
        
        self.weights = [1.0/self.num_particles] * self.num_particles
    
    def predict(self, odom_dx, odom_dy, odom_dtheta):
        """Motion model: update particles based on odometry"""
        for i in range(self.num_particles):
            # Add noise to odometry
            dx = odom_dx + np.random.normal(0, 0.05)
            dy = odom_dy + np.random.normal(0, 0.05)
            dtheta = odom_dtheta + np.random.normal(0, 0.03)
            
            # Update particle pose
            self.particles[i][0] += dx * np.cos(self.particles[i][2]) - dy * np.sin(self.particles[i][2])
            self.particles[i][1] += dx * np.sin(self.particles[i][2]) + dy * np.cos(self.particles[i][2])
            self.particles[i][2] += dtheta
    
    def update_weights(self, laser_scan):
        """Measurement model: update weights based on laser scan"""
        for i in range(self.num_particles):
            # For each particle, compute expected laser readings
            # Compare with actual laser scan
            # Weight particle based on match quality
            
            likelihood = self.compute_likelihood(self.particles[i], laser_scan)
            self.weights[i] *= likelihood
        
        # Normalize weights
        self.weights = np.array(self.weights)
        self.weights /= np.sum(self.weights)
    
    def resample(self):
        """Low variance resampling"""
        new_particles = []
        M = len(self.particles)
        r = np.random.uniform(0, 1.0/M)
        c = self.weights[0]
        i = 0
        
        for m in range(M):
            u = r + m * (1.0/M)
            while u > c:
                i += 1
                c += self.weights[i]
            new_particles.append(self.particles[i].copy())
        
        self.particles = new_particles
        self.weights = [1.0/M] * M
    
    def estimate_pose(self):
        """Return weighted average of particles"""
        avg_x = np.average([p[0] for p in self.particles], weights=self.weights)
        avg_y = np.average([p[1] for p in self.particles], weights=self.weights)
        avg_theta = np.average([p[2] for p in self.particles], weights=self.weights)
        
        return avg_x, avg_y, avg_theta
