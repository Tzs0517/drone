import numpy as np

np.set_printoptions(precision=4, suppress=True)

# 从三张截图里取的稳定行（格式: ax, ay, az）
# 平放
flat  = np.array([4157.0, -226.0, -374.0])
# 抬头 30 度（取一个较稳定的行）
pitch = np.array([3093.0, -2832.0, -1016.0])
# 右滚 30 度
roll  = np.array([2797.0, -905.0, -3150.0])

def unit(v):
    return v / np.linalg.norm(v)

a1, a2, a3 = unit(flat), unit(pitch), unit(roll)

print("传感器测到的'上方'方向（归一化）：")
print("  平放   a1 =", a1, " |a|=", np.linalg.norm(flat) / 4096.0, "g")
print("  抬头   a2 =", a2, " |a|=", np.linalg.norm(pitch) / 4096.0, "g")
print("  右滚   a3 =", a3, " |a|=", np.linalg.norm(roll) / 4096.0, "g")
print()
print("两两夹角（度）：")
for n, (x, y) in [("a1-a2", (a1, a2)), ("a1-a3", (a1, a3)), ("a2-a3", (a2, a3))]:
    print("  %s = %.1f" % (n, np.degrees(np.arccos(np.clip(x @ y, -1, 1)))))

# 机体坐标系下的"上方"（NED: x前 y右 z下 -> 上方 = -z）
# 平放: (0,0,-1)
# 抬头30: 绕 +y(右) 转 -30 度 => 上方向 = (0.5, 0, -0.866)
# 右滚30: 绕 +x(前) 转 +30 度 => 上方向 = (0,-0.5,-0.866)
u1 = np.array([0.0, 0.0, -1.0])
u2 = np.array([np.sin(np.radians(30)), 0.0, -np.cos(np.radians(30))])
u3 = np.array([0.0, -np.sin(np.radians(30)), -np.cos(np.radians(30))])

print()
print("机体坐标系下应有的'上方'：")
print("  平放", u1, " 抬头", u2, " 右滚", u3)
print("  机体下两两夹角: u1-u2=%.1f  u1-u3=%.1f  u2-u3=%.1f" % (
    np.degrees(np.arccos(np.clip(u1 @ u2, -1, 1))),
    np.degrees(np.arccos(np.clip(u1 @ u3, -1, 1))),
    np.degrees(np.arccos(np.clip(u2 @ u3, -1, 1)))))

# Kabsch: 求旋转 R 使 R @ u_i ≈ a_i  (机体 -> 传感器)
U = np.column_stack([u1, u2, u3])
A = np.column_stack([a1, a2, a3])
H = U @ A.T
Uu, S, Vt = np.linalg.svd(H)
d = np.sign(np.linalg.det(Vt.T @ Uu.T))
D = np.diag([1.0, 1.0, d])
R = Vt.T @ D @ Uu.T          # 机体 -> 传感器

print()
print("最小二乘解出的 机体->传感器 旋转矩�阵 R：")
print(R)
print("  det =", np.linalg.det(R))
res = [np.linalg.norm(R @ u - a) for u, a in [(u1, a1), (u2, a2), (u3, a3)]]
print("  三组残差:", ["%.4f" % x for x in res])

print()
print("=" * 70)
print("把 R 四舍五入成「置换+符号」的形式（工程上真正要用的映射）：")
M = np.round(R, 0)
print(M)
print("  每一行是「机体的某个轴由传感器哪个轴构成」")
names = ["机头(前)", "右翼(右)", "机腹(下)"]
snames = ["sx", "sy", "sz"]
for i in range(3):
    idx = int(np.argmax(np.abs(M[i])))
    print("    %s = %s %s" % (names[i], "+" if M[i][idx] > 0 else "-", snames[idx]))

print()
print("把上面的映射反着用（传感器 -> 机体），并对照第 8 课的算法约定")
print("算法要求：静止水平时喂进去的加速度应该是 (0,0,+1)")
Minv = np.linalg.inv(M)
for name, a in [("平放", a1), ("抬头", a2), ("右滚", a3)]:
    body = Minv @ a
    # 算法约定: a_alg = C^T e_z, 水平时应为 (0,0,1) => 需要取反
    alg = -body
    print("  %s: 传感器%s -> 机体系%s -> 喂给算法%s" % (
        name, np.round(a, 3), np.round(body, 3), np.round(alg, 3)))
