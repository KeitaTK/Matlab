"""
OBSV_data.csv の TimeUS フィールドを詳しく分析
100Hz想定での妥当性を検証
"""
import csv
from collections import defaultdict

data_path = 'c:/Users/taki/Local/local/Matlab/OBS/phase_estimation_results/data/OBSV_data.csv'

print("=" * 100)
print("OBSV_data.csv TimeUS フィールド分析")
print("=" * 100)

timeus_values = []
with open(data_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            timeus = float(row['TimeUS'])
            timeus_values.append(timeus)
        except ValueError:
            pass

print(f"\n【基本統計】")
print(f"総データ行数: {len(timeus_values)}")
print(f"TimeUS最小値: {min(timeus_values):.2e}")
print(f"TimeUS最大値: {max(timeus_values):.2e}")
print(f"TimeUS範囲: {(max(timeus_values) - min(timeus_values)):.2e}")

# マイクロ秒と仮定した場合の分析
print(f"\n【マイクロ秒と仮定した場合】")
range_us = max(timeus_values) - min(timeus_values)
range_sec = range_us / 1e6
range_min = range_sec / 60
range_hour = range_min / 60
range_day = range_hour / 24
print(f"時間範囲: {range_sec:.2f} 秒 = {range_min:.2f} 分 = {range_hour:.2f} 時間 = {range_day:.2f} 日")

# 100Hzでの期待値
expected_period_us = 10000  # 100Hzなら10msごと
print(f"\n【100Hz仮定での期待値】")
print(f"予想サンプリング周期: {expected_period_us} μs (10 ms)")
print(f"予想総データ行数（範囲から計算）: {range_us / expected_period_us:.0f}")

# 連続データの時間差を分析
print(f"\n【TimeUS値の差分分析】")
diffs = []
diffs_valid = []
for i in range(1, min(21, len(timeus_values))):  # 最初の20行の差分を表示
    diff = timeus_values[i] - timeus_values[i-1]
    diffs.append(diff)
    if diff > 0:
        diffs_valid.append(diff)
    print(f"行{i:2d}: {timeus_values[i-1]:.2e} → {timeus_values[i]:.2e}, 差分: {diff:.2e} μs ({diff/1000:.2f} ms)")

if diffs_valid:
    print(f"\n【差分統計（正の差分のみ）】")
    print(f"正の差分数: {len(diffs_valid)} / {len(diffs)}")
    print(f"最小差分: {min(diffs_valid):.2e} μs")
    print(f"最大差分: {max(diffs_valid):.2e} μs")
    print(f"平均差分: {sum(diffs_valid)/len(diffs_valid):.2e} μs")

# データが時系列順になっているか確認
is_monotonic = all(timeus_values[i] <= timeus_values[i+1] for i in range(len(timeus_values)-1))
print(f"\n【時系列順序】")
print(f"厳密に昇順: {is_monotonic}")

# 昇順のセクションを検出
print(f"\n【昇順セクションの検出】")
sections = []
current_section = [0]
for i in range(1, len(timeus_values)):
    if timeus_values[i] > timeus_values[i-1]:
        current_section.append(i)
    else:
        if len(current_section) > 1:
            sections.append(current_section)
        current_section = [i]
if len(current_section) > 1:
    sections.append(current_section)

print(f"昇順セクション数: {len(sections)}")
for idx, section in enumerate(sections[:5]):  # 最初の5セクション
    start_idx = section[0]
    end_idx = section[-1]
    start_time = timeus_values[start_idx]
    end_time = timeus_values[end_idx]
    duration = end_time - start_time
    num_points = len(section)
    avg_period = duration / (num_points - 1) if num_points > 1 else 0
    print(f"  セクション{idx+1}: 行{start_idx}-{end_idx} ({num_points}点), "
          f"時間範囲: {duration:.2e}μs, 平均周期: {avg_period:.2e}μs ({avg_period/1000:.3f}ms)")

# 考察
print(f"\n{'='*100}")
print("【考察】")
print(f"{'='*100}")
print("""
1. TimeUS は確かにマイクロ秒単位に見えるが、値が大きすぎる
   - 1.5e15～2.0e15 μs = 1500～2000秒 ≈ 25～33分程度？
   
   実際には、TimeUSが本来のマイクロ秒タイムスタンプ（通常は起動からの相対時間）
   ではなく、別の形式データ（例：ナノ秒の一部など）の可能性あり

2. データが時系列順に並んでいない
   - 複数の昇順セクションがある
   - これは異なるセッション/試行のデータが混在している可能性

3. 100Hzサンプリングの検証
   - 期待値: 10000 μs (10 ms) の周期
   - 実際: セクション内での平均周期をチェック必要
   
4. データの可視化
   - TimeUS値そのものよりも、行番号を時間軸とした方が意味がある可能性
   - または、セクション内での相対時刻を計算する必要がある
""")

print(f"{'='*100}\n")
