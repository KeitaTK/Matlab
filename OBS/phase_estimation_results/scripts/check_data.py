"""
OBSV_data.csv のデータ内容を確認するスクリプト
"""
import pandas as pd
import numpy as np
import os

# データファイルのパス
data_path = os.path.join('..', 'data', 'OBSV_data.csv')

print("=" * 80)
print("OBSV_data.csv データ確認")
print("=" * 80)

# CSVファイルを読み込む
try:
    df = pd.read_csv(data_path)
    print(f"\n✓ ファイル読み込み成功")
    print(f"  データ行数: {len(df)}")
    print(f"  カラム数: {len(df.columns)}")
    
    print("\n" + "-" * 80)
    print("カラム名:")
    print("-" * 80)
    for i, col in enumerate(df.columns, 1):
        print(f"{i:2d}. {col}")
    
    print("\n" + "-" * 80)
    print("各カラムの統計情報:")
    print("-" * 80)
    
    for col in df.columns:
        values = df[col].dropna()
        if len(values) > 0:
            print(f"\n{col}:")
            print(f"  非NaN値数: {len(values)}")
            print(f"  最小値: {values.min():.6e}")
            print(f"  最大値: {values.max():.6e}")
            print(f"  平均値: {values.mean():.6e}")
            print(f"  標準偏差: {values.std():.6e}")
    
    print("\n" + "-" * 80)
    print("最初の5行:")
    print("-" * 80)
    print(df.head())
    
    print("\n" + "-" * 80)
    print("データ型:")
    print("-" * 80)
    print(df.dtypes)
    
    print("\n" + "-" * 80)
    print("欠損値の確認:")
    print("-" * 80)
    print(df.isnull().sum())
    
    # TimeUSを時間に変換してみる
    if 'TimeUS' in df.columns:
        print("\n" + "-" * 80)
        print("時間情報:")
        print("-" * 80)
        time_us = df['TimeUS'].dropna()
        if len(time_us) > 1:
            time_range = (time_us.max() - time_us.min()) / 1e6  # マイクロ秒から秒へ
            print(f"  時間範囲: {time_range:.2f} 秒")
            print(f"  開始時刻: {time_us.min()}")
            print(f"  終了時刻: {time_us.max()}")
    
    print("\n" + "=" * 80)
    print("データ確認完了")
    print("=" * 80)
    
except Exception as e:
    print(f"\n✗ エラー発生: {e}")
    import traceback
    traceback.print_exc()
