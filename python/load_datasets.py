"""
Simple dataset loader script
This is a basic script - use dataset_loader.py for more features
"""
import pandas as pd
import os

def load_datasets():
    """Load all CSV datasets"""
    base_path = os.path.join('..', 'Datasets', 'csv')
    
    datasets = {}
    
    try:
        datasets['als'] = pd.read_csv(os.path.join(base_path, 'ALS.csv'))
        datasets['justa'] = pd.read_csv(os.path.join(base_path, 'Justa.csv'))
        datasets['synth2'] = pd.read_csv(os.path.join(base_path, 'Synth2.csv'))
        datasets['synth3'] = pd.read_csv(os.path.join(base_path, 'Synth3.csv'))
    except FileNotFoundError as e:
        print(f"Error loading datasets: {e}")
        print(f"Make sure CSV files are in {base_path}")
        
    return datasets

# Example usage
if __name__ == '__main__':
    datasets = load_datasets()
    
    for name, df in datasets.items():
        print(f"\n{name.upper()} Dataset:")
        print(f"  Samples: {len(df)}")
        print(f"  Columns: {list(df.columns)}")
        if len(df) > 0:
            print(f"  Duration: {df['time'].iloc[-1]:.2f} seconds")
            print(f"  First row:\n{df.iloc[0]}")