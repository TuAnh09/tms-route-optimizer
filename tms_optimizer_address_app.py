# tms_optimizer_full_app.py
"""
TMS Route Optimizer (Name + Address + Latitude + Longitude)
- Input: warehouse + delivery list (name, address, lat, lon)
- Optimize route using OR-Tools (if available) or Nearest Neighbor
"""

import streamlit as st
import pandas as pd
import numpy as np
import math

try:
    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp
    ORTOOLS_AVAILABLE = True
except Exception:
    ORTOOLS_AVAILABLE = False

# ---------- CONFIG ----------
st.set_page_config(page_title="TMS Tối ưu tuyến đường (Full Info)", layout="wide")
st.title("🚚 Ứng dụng TMS tối ưu hóa tuyến đường (Tên + Địa chỉ + Tọa độ)")

# ---------- HÀM HỖ TRỢ ----------
def haversine(lat1, lon1, lat2, lon2):
    """Tính khoảng cách giữa 2 điểm theo công thức haversine (km)"""
    R = 6371
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) *
         math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def compute_distance_matrix(coords):
    n = len(coords)
    dist = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                dist[i][j] = int(haversine(coords[i][0], coords[i][1],
                                           coords[j][0], coords[j][1]) * 1000)
    return dist

def solve_with_ortools(distance_matrix):
    n = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        f, t = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        return distance_matrix[f][t]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.time_limit.seconds = 5

    solution = routing.SolveWithParameters(params)
    if not solution:
        return None, None

    index = routing.Start(0)
    route = []
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    route.append(manager.IndexToNode(index))

    total_distance = 0
    for i in range(len(route) - 1):
        total_distance += distance_matrix[route[i]][route[i + 1]]
    return route, total_distance

def nearest_neighbor(distance_matrix):
    n = len(distance_matrix)
    visited = [False] * n
    route = [0]
    visited[0] = True
    total = 0
    current = 0
    for _ in range(n - 1):
        next_node = None
        min_d = float('inf')
        for j in range(n):
            if not visited[j] and distance_matrix[current][j] < min_d:
                min_d = distance_matrix[current][j]
                next_node = j
        if next_node is None:
            break
        route.append(next_node)
        total += min_d
        visited[next_node] = True
        current = next_node
    route.append(0)
    total += distance_matrix[current][0]
    return route, total

# ---------- INPUT ----------
with st.sidebar:
    st.header("⚙️ Cấu hình")
    avg_speed = st.number_input("Tốc độ trung bình (km/h)", 5.0, 120.0, 30.0)
    allow_return = st.checkbox("Trở lại kho cuối chuyến", value=True)

st.subheader("1️⃣ Nhập thông tin kho hàng")
warehouse_name = st.text_input("Tên kho", "Kho Trung tâm")
warehouse_address = st.text_input("Địa chỉ kho", "285 Cách Mạng Tháng 8, Quận 10, TP.HCM")
warehouse_lat = st.number_input("Vĩ độ (lat) kho", value=10.7765)
warehouse_lon = st.number_input("Kinh độ (lon) kho", value=106.7009)

st.subheader("2️⃣ Danh sách điểm giao hàng")
st.markdown("Nhập thủ công hoặc tải file CSV có cột: `Tên,Địa chỉ,Lat,Lon`")

sample = """Khách A,43 Nguyễn Huệ,Quận 1,10.7769,106.7009
Khách B,1 Lê Duẩn,Quận 1,10.7792,106.6998
Khách C,60 Lý Tự Trọng,Quận 1,10.7754,106.7032
Khách D,500 Điện Biên Phủ,Quận 3,10.7797,106.6896"""
addresses_text = st.text_area("Nhập danh sách (CSV)", sample, height=180)

uploaded = st.file_uploader("Hoặc tải file CSV", type=["csv"])

if uploaded:
    df = pd.read_csv(uploaded)
else:
    from io import StringIO
    df = pd.read_csv(StringIO(addresses_text), names=["Tên", "Địa chỉ", "Khu vực", "Lat", "Lon"])

if st.button("🚀 Tối ưu hóa tuyến"):
    # Tổng hợp dữ liệu
    all_names = [warehouse_name] + df["Tên"].tolist()
    all_addresses = [warehouse_address] + df["Địa chỉ"].fillna("").tolist()
    all_coords = [(warehouse_lat, warehouse_lon)] + list(zip(df["Lat"], df["Lon"]))

    dist_matrix = compute_distance_matrix(all_coords)

    with st.spinner("Đang tối ưu hóa tuyến..."):
        if ORTOOLS_AVAILABLE:
            route, total = solve_with_ortools(dist_matrix)
        else:
            route, total = nearest_neighbor(dist_matrix)

    if route is None:
        st.error("Không tìm được lời giải tối ưu.")
        st.stop()

    if not allow_return and route[-1] == 0:
        route = route[:-1]

    total_km = total / 1000
    time_h = total_km / avg_speed
    st.success(f"Tổng quãng đường: **{total_km:.2f} km**, Thời gian ước tính: **{time_h:.2f} giờ**")

    route_names = [all_names[i] for i in route]
    st.write("**Thứ tự tuyến:**")
    st.write(" → ".join(route_names))

    df_result = pd.DataFrame({
        "Thứ tự": list(range(1, len(route) + 1)),
        "Tên": [all_names[i] for i in route],
        "Địa chỉ": [all_addresses[i] for i in route],
        "Lat": [all_coords[i][0] for i in route],
        "Lon": [all_coords[i][1] for i in route],
    })
    st.dataframe(df_result)

    # Map visualization
    import pydeck as pdk
    points = [{"lat": c[0], "lon": c[1], "name": all_names[i]} for i, c in enumerate(all_coords)]
    path = [{"path": [(all_coords[i][1], all_coords[i][0]) for i in route]}]
    view = pdk.ViewState(latitude=np.mean([c[0] for c in all_coords]),
                         longitude=np.mean([c[1] for c in all_coords]),
                         zoom=12)
    layer_route = pdk.Layer("PathLayer", path, get_path="path", get_width=4, width_scale=20, get_color=[0, 128, 255])
    layer_points = pdk.Layer("ScatterplotLayer", points, get_position=["lon","lat"], get_radius=80,
                             get_fill_color=[255, 128, 0], pickable=True)
    r = pdk.Deck(layers=[layer_route, layer_points], initial_view_state=view, tooltip={"text": "{name}"})
    st.pydeck_chart(r)

    st.download_button("⬇️ Tải kết quả CSV", df_result.to_csv(index=False).encode("utf-8"),
                       "route_result.csv", "text/csv")

st.caption("💡 Lưu ý: Dữ liệu cần có các cột: Tên, Địa chỉ, Lat, Lon. Có thể nhập trực tiếp hoặc tải file CSV.")
