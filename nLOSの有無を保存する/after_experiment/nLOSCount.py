import streamlit as st
import pandas as pd
import plotly.express as px
import json
from collections import Counter
import math # 3D座標計算のためにmathライブラリをインポート

# --- UIの初期設定 ---
st.set_page_config(layout="wide", page_title="UWBセンサーデータ分析")
st.title('UWBセンサーデータ分析ダッシュボード 📊')
st.sidebar.header('⚙️ 設定')

# --- ファイルアップローダー ---
uploaded_file = st.sidebar.file_uploader(
    "分析したいCSVファイルをアップロードしてください",
    type=['csv']
)

# --- データ読み込み関数 ---
@st.cache_data
def load_data(file):
    try:
        df = pd.read_csv(file)
        if df.shape[1] == 1:
            file.seek(0)
            df = pd.read_csv(file, sep='\t')
        if 'remarks' in df.columns:
            df['remarks'] = df['remarks'].fillna('')
        return df
    except Exception as e:
        st.error(f"ファイルの読み込みに失敗しました: {e}")
        return None

# --- メイン処理 ---
if uploaded_file is not None:
    df = load_data(uploaded_file)

    if df is not None:
        # --- フィルター機能 ---
        st.sidebar.subheader('絞り込みフィルター')
        orientation_options = ['すべて'] + df['uwb_orientation_condition'].unique().tolist()
        nlos_options = ['すべて'] + df['nlos_los_expected'].unique().tolist()
        selected_orientation = st.sidebar.selectbox('UWBの向き:', orientation_options)
        selected_nlos = st.sidebar.selectbox('見通し(Expected):', nlos_options)
        filtered_df = df.copy()
        if selected_orientation != 'すべて':
            filtered_df = filtered_df[filtered_df['uwb_orientation_condition'] == selected_orientation]
        if selected_nlos != 'すべて':
            filtered_df = filtered_df[filtered_df['nlos_los_expected'] == selected_nlos]

        # --- 実験選択機能 (Prev/Nextボタン付き) ---
        st.sidebar.subheader('表示する実験を選択')
        if not filtered_df.empty:
            filtered_df['display_option'] = filtered_df['experiment_no'].astype(str) + " : " + filtered_df['remarks'].astype(str)
            options_list = filtered_df['display_option'].tolist()
            if 'selected_index' not in st.session_state or st.session_state.selected_index >= len(options_list):
                st.session_state.selected_index = 0
            col1, col2 = st.sidebar.columns(2)
            if col1.button('◀️ 前の実験へ', use_container_width=True):
                st.session_state.selected_index = max(0, st.session_state.selected_index - 1)
            if col2.button('次の実験へ ▶️', use_container_width=True):
                st.session_state.selected_index = min(len(options_list) - 1, st.session_state.selected_index + 1)
            
            selected_display_option = st.selectbox(
                '実験内容:', options_list, index=st.session_state.selected_index, key='experiment_selector'
            )
            st.session_state.selected_index = options_list.index(selected_display_option)
            selected_exp_no = int(selected_display_option.split(' : ')[0])
            selected_data = df[df['experiment_no'] == selected_exp_no].iloc[0]

            # --- メイン画面 ---
            st.header(f'🔬 実験番号: {selected_exp_no} の分析結果')
            with st.expander("詳細な実験条件を表示", expanded=True):
                cond_col1, cond_col2, cond_col3 = st.columns(3)
                cond_col1.metric("📏 設定距離 (m)", selected_data['distance_m'])
                cond_col2.metric("📡 UWBの向き", selected_data['uwb_orientation_condition'])
                cond_col3.metric("🔭 見通し(想定)", selected_data['nlos_los_expected'])
                st.info(f"**備考:** {selected_data['remarks']}")
            
            # --- 解析とサマリー ---
            try:
                raw_frames = json.loads(selected_data['raw_frames_data'])
                nlos_los_list = [d['TWR0']['nlos_los'] for d in raw_frames if 'TWR0' in d]
                nlos_los_counts = Counter(nlos_los_list)
                st.subheader('📋 計測サマリー')
                sum_col1, sum_col2, sum_col3, sum_col4 = st.columns(4)
                sum_col1.metric("✅ LOS", nlos_los_counts.get('LOS', 0), help="Line-of-Sight (見通し内) の回数")
                sum_col2.metric("❌ NLOS", nlos_los_counts.get('NLOS', 0), help="Non-Line-of-Sight (見通し外) の回数")
                sum_col3.metric("🔢 総フレーム数", len(raw_frames))
                sum_col4.metric("⏱️ 計測時間 (ms)", selected_data['measurement_time_ms'])

                # --- データ抽出と3D座標計算 ---
                distances = [d['TWR0']['distance'] for d in raw_frames if 'TWR0' in d]
                h_angles_deg = [d['TWR0']['horizontal_angle'] for d in raw_frames if 'TWR0' in d]
                e_angles_deg = [d['TWR0']['elevation_angle'] for d in raw_frames if 'TWR0' in d]
                
                x_coords, y_coords, z_coords = [], [], []
                for dist, h_angle_d, e_angle_d in zip(distances, h_angles_deg, e_angles_deg):
                    h_angle_r = math.radians(h_angle_d) # 水平角をラジアンに変換
                    e_angle_r = math.radians(e_angle_d) # 仰角をラジアンに変換
                    
                    # 球面座標から直交座標へ変換
                    x = dist * math.cos(e_angle_r) * math.cos(h_angle_r)
                    y = dist * math.cos(e_angle_r) * math.sin(h_angle_r)
                    z = dist * math.sin(e_angle_r)
                    x_coords.append(x)
                    y_coords.append(y)
                    z_coords.append(z)

                plot_df = pd.DataFrame({
                    'Distance (m)': distances,
                    'Horizontal Angle (°)': h_angles_deg,
                    'Elevation Angle (°)': e_angles_deg,
                    'NLOS/LOS': nlos_los_list,
                    'x': x_coords, 'y': y_coords, 'z': z_coords
                })

                # --- グラフ表示 (4列レイアウト) ---
                st.subheader('📈 分布グラフ')
                g_col1, g_col2, g_col3, g_col4 = st.columns(4)
                color_map = {'LOS':'blue', 'NLOS':'red'}

                with g_col1:
                    fig_dist = px.histogram(plot_df, x='Distance (m)', color='NLOS/LOS', title='距離', marginal='box', color_discrete_map=color_map)
                    st.plotly_chart(fig_dist, use_container_width=True)
                with g_col2:
                    fig_h_angle = px.histogram(plot_df, x='Horizontal Angle (°)', color='NLOS/LOS', title='水平角', marginal='box', color_discrete_map=color_map)
                    st.plotly_chart(fig_h_angle, use_container_width=True)
                with g_col3:
                    fig_e_angle = px.histogram(plot_df, x='Elevation Angle (°)', color='NLOS/LOS', title='仰角', marginal='box', color_discrete_map=color_map)
                    st.plotly_chart(fig_e_angle, use_container_width=True)
                with g_col4:
                    fig_3d = px.scatter_3d(plot_df, x='x', y='y', z='z', color='NLOS/LOS', title='3D空間分布', color_discrete_map=color_map)
                    fig_3d.update_traces(marker=dict(size=3)) # マーカーサイズを調整
                    st.plotly_chart(fig_3d, use_container_width=True)

                # --- グラフの見方の解説 ---
                with st.expander("💡 グラフの見方"):
                    st.markdown("""
                        - **分布図 (距離・水平角・仰角)**: 各測定値がどのような範囲に、どのくらいの頻度で分布しているかを示します。
                            - **山が急で幅が狭い**ほど、測定値が安定していることを意味します。
                            - **色分け**は <span style='color:blue;'>■</span> **LOS (見通し内)** と <span style='color:red;'>■</span> **NLOS (見通し外)** です。
                        - **3D空間分布**: センサーを原点(0,0,0)としたとき、測定された各点が空間内のどこに位置するかをプロットしたものです。
                            - 点群が**一箇所にキュッとまとまっている**ほど、3次元空間での測位が安定していることを示します。
                            - **マウスでドラッグ**すると、グラフを回転させて様々な角度から確認できます。
                    """, unsafe_allow_html=True)

            except (json.JSONDecodeError, TypeError, KeyError) as e:
                st.error(f'raw_frames_dataの解析に失敗しました。データ形式が想定と異なる可能性があります。エラー: {e}')
        else:
            st.warning('フィルター条件に一致する実験がありません。')
else:
    st.info('👈 サイドバーから分析するCSVファイルをアップロードしてください。')