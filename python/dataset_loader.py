"""
Dataset loader for AHRS filter testing
Reads CSV datasets and provides data in appropriate format
"""
import pandas as pd
import numpy as np
import os


class DatasetLoader:
    """
    Load and manage AHRS datasets from CSV files
    """
    
    def __init__(self, base_path='Datasets/csv'):
        self.base_path = base_path
        self.datasets = {}
        
    def load_dataset(self, dataset_name):
        """
        Load a specific dataset from CSV
        
        Args:
            dataset_name: Name of the dataset ('ALS', 'Justa', 'Synth2', 'Synth3')
            
        Returns:
            Dictionary containing time, sensor data, and reference quaternions
        """
        file_map = {
            'ALS': 'ALS.csv',
            'Justa': 'Justa.csv',
            'Synth2': 'Synth2.csv',
            'Synth3': 'Synth3.csv'
        }
        
        if dataset_name not in file_map:
            raise ValueError(f"Unknown dataset: {dataset_name}")
            
        file_path = os.path.join(self.base_path, file_map[dataset_name])
        
        # Read CSV file
        df = pd.read_csv(file_path)
        
        # Extract data
        data = {
            'time': df['time'].values,
            'gyroscope': df[['gx', 'gy', 'gz']].values,
            'accelerometer': df[['ax', 'ay', 'az']].values,
            'magnetometer': df[['mx', 'my', 'mz']].values,
            'reference': df[['ref1', 'ref2', 'ref3', 'ref4']].values
        }
        
        self.datasets[dataset_name] = data
        return data
        
    def load_all_datasets(self):
        """
        Load all available datasets
        
        Returns:
            Dictionary with all datasets
        """
        for name in ['ALS', 'Justa', 'Synth2', 'Synth3']:
            try:
                self.load_dataset(name)
            except Exception as e:
                print(f"Warning: Could not load {name}: {e}")
                
        return self.datasets
        
    def get_dataset_segment(self, dataset_name, start_idx=None, end_idx=None):
        """
        Get a segment of a dataset
        
        Args:
            dataset_name: Name of the dataset
            start_idx: Starting index (None = beginning)
            end_idx: Ending index (None = end)
            
        Returns:
            Dictionary containing the data segment
        """
        if dataset_name not in self.datasets:
            self.load_dataset(dataset_name)
            
        data = self.datasets[dataset_name]
        
        if start_idx is None:
            start_idx = 0
        if end_idx is None:
            end_idx = len(data['time'])
            
        segment = {
            'time': data['time'][start_idx:end_idx],
            'gyroscope': data['gyroscope'][start_idx:end_idx],
            'accelerometer': data['accelerometer'][start_idx:end_idx],
            'magnetometer': data['magnetometer'][start_idx:end_idx],
            'reference': data['reference'][start_idx:end_idx]
        }
        
        return segment
