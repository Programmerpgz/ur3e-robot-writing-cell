-- UR3e pise "Blaz Rodic" - CoppeliaSim (Customization skript, CoppeliaSim 4.6+).
-- Skripta pri otvaranju scene JEDNOM izgradi trajnu celiju (robot iz DH tablice, stol, papir,
-- SCHUNK hvataljka s markerom, kompresor, FRL, elektromagnetski ventil). Objekti se spremaju u .ttt
-- i ostaju u sceni i bez simulacije. Pritiskom na Play robot (FK + numericka IK iz DH tablice)
-- uzima marker (ventil DO0) i ispisuje tekst na papir.
sim = require('sim')

-- ---- parametri projekta ---------------------------------------------------
local TEXT_NAME = 'Blaz Rodic'              -- samo informativno (glifovi su dolje)
local TOOL_Z = 0.14                         -- prirubnica -> vrh markera [m]
local SPEED_WRITE, SPEED_MOVE = 0.06, 0.25  -- brzina TCP-a [m/s] (limit UR3e je 3 m/s)
local Z_WRITE, Z_UP = 0.002, 0.030          -- visina vrha markera: pisanje / podignuto [m]
local X0, Y0 = 0.20, 0.1525                 -- pocetak teksta (Base sustav): baza slova, lijevi rub
local SCALE = 0.005                         -- 1 jedinica fonta = 5 mm (visina velikog slova 50 mm)
local P_TANK, P_MIN = 6.0, 2.5              -- nazivni i minimalni radni tlak hvataljke [bar]
local VALVE_STEPS = 4                       -- kasnjenje ventila u koracima (stvarno oko 0.015 s)
local GAP_OPEN, GAP_PEN, FINGER_T = 0.016, 0.008, 0.008   -- raspon prstiju [m]
local Q_SEED = {0.0, -1.2, 1.8, -2.2, -math.pi / 2, 0.0}
local INK = {0.05, 0.1, 0.8}

-- ---- DH parametri UR3e (UR support article), metri i radijani ------------
local DH_A = {0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0}
local DH_D = {0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921}
local DH_AL = {math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0}

-- ---- matematika -----------------------------------------------------------
local function eye()
    return {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}
end

local function mmul(A, B)
    local C = {}
    for i = 1, 4 do
        C[i] = {}
        for j = 1, 4 do
            local s = 0
            for k = 1, 4 do s = s + A[i][k] * B[k][j] end
            C[i][j] = s
        end
    end
    return C
end

local function dh(th, d, a, al)
    local ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return {{ct, -st * ca, st * sa, a * ct},
            {st, ct * ca, -ct * sa, a * st},
            {0, sa, ca, d},
            {0, 0, 0, 1}}
end

local function tf(x, y, z, yaw)
    local c, s = math.cos(yaw or 0), math.sin(yaw or 0)
    return {{c, -s, 0, x}, {s, c, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}}
end

local function fk(q)
    local T = eye()
    for i = 1, 6 do T = mmul(T, dh(q[i], DH_D[i], DH_A[i], DH_AL[i])) end
    for i = 1, 3 do T[i][4] = T[i][4] + TOOL_Z * T[i][3] end    -- alat duz osi z prirubnice
    return T
end

local function rotErr(Rt, Rc)
    local Re = {}
    for i = 1, 3 do
        Re[i] = {}
        for j = 1, 3 do
            local s = 0
            for k = 1, 3 do s = s + Rt[i][k] * Rc[j][k] end
            Re[i][j] = s
        end
    end
    return {0.5 * (Re[3][2] - Re[2][3]), 0.5 * (Re[1][3] - Re[3][1]), 0.5 * (Re[2][1] - Re[1][2])}
end

local function poseError(Tt, T)
    local r = rotErr(Tt, T)
    return {Tt[1][4] - T[1][4], Tt[2][4] - T[2][4], Tt[3][4] - T[3][4], r[1], r[2], r[3]}
end

local function norm(v)
    local s = 0
    for i = 1, #v do s = s + v[i] * v[i] end
    return math.sqrt(s)
end

local function jacobian(q)
    local J = {}
    for r = 1, 6 do J[r] = {} end
    local eps = 1e-6
    local T0 = fk(q)
    for c = 1, 6 do
        local qd = {q[1], q[2], q[3], q[4], q[5], q[6]}
        qd[c] = qd[c] + eps
        local e = poseError(fk(qd), T0)
        for r = 1, 6 do J[r][c] = e[r] / eps end
    end
    return J
end

local function solve(M, b)        -- Gaussova eliminacija s pivotiranjem, M je 6x6
    local n = #b
    local A = {}
    for i = 1, n do
        A[i] = {}
        for j = 1, n do A[i][j] = M[i][j] end
        A[i][n + 1] = b[i]
    end
    for c = 1, n do
        local p = c
        for r = c + 1, n do if math.abs(A[r][c]) > math.abs(A[p][c]) then p = r end end
        A[c], A[p] = A[p], A[c]
        for r = c + 1, n do
            local f = A[r][c] / A[c][c]
            for k = c, n + 1 do A[r][k] = A[r][k] - f * A[c][k] end
        end
    end
    local x = {}
    for i = n, 1, -1 do
        local s = A[i][n + 1]
        for k = i + 1, n do s = s - A[i][k] * x[k] end
        x[i] = s / A[i][i]
    end
    return x
end

local function ik(Tt, q0)          -- prigusena metoda najmanjih kvadrata, start iz q0
    local q = {q0[1], q0[2], q0[3], q0[4], q0[5], q0[6]}
    local lam = 0.05
    for _ = 1, 200 do
        local e = poseError(Tt, fk(q))
        if norm(e) < 1e-5 then return q, true end
        local J = jacobian(q)
        local M = {}
        for i = 1, 6 do
            M[i] = {}
            for j = 1, 6 do
                local s = 0
                for k = 1, 6 do s = s + J[i][k] * J[j][k] end
                M[i][j] = s + (i == j and lam * lam or 0)
            end
        end
        local y = solve(M, e)
        for i = 1, 6 do
            local dq = 0
            for k = 1, 6 do dq = dq + J[k][i] * y[k] end
            q[i] = q[i] + dq
        end
    end
    return q, norm(poseError(Tt, fk(q))) < 1e-4
end

-- alat gleda okomito dolje; cilj je zadan u UR "Base" sustavu (x naprijed),
-- a DH sustav baze je zarotiran 180 deg oko z, pa se prva dva retka mijenjaju predznak
local function targetDown(x, y, z)
    return {{-1, 0, 0, -x}, {0, 1, 0, -y}, {0, 0, -1, z}, {0, 0, 0, 1}}
end

local function linePath(q0, p0, p1, dt, speed)
    local dist = math.sqrt((p1[1] - p0[1]) ^ 2 + (p1[2] - p0[2]) ^ 2 + (p1[3] - p0[3]) ^ 2)
    local n = math.max(1, math.ceil(dist / (speed * dt)))
    local q, path = q0, {}
    for k = 1, n do
        local f = k / n
        local p = {p0[1] + (p1[1] - p0[1]) * f, p0[2] + (p1[2] - p0[2]) * f, p0[3] + (p1[3] - p0[3]) * f}
        local ok
        q, ok = ik(targetDown(p[1], p[2], p[3]), q)
        if not ok then error(string.format('IK ne konvergira u tocki %.3f %.3f %.3f', p[1], p[2], p[3])) end
        path[#path + 1] = {q = q, p = p}
    end
    return path
end

-- ---- jednoliniski font "Blaz Rodic" s dijakriticima (jedinica: 1 = 3 mm, y prema gore) ----
-- svaki glif: sirina, popis poteza; potez = niz tocaka {x, y}
local GLYPH = {
    ['B'] = {w = 6, {{0,0},{0,10},{4.5,10},{5.5,9},{5.5,6},{4.5,5},{0,5}}, {{4.5,5},{6,4},{6,1},{5,0},{0,0}}},
    ['l'] = {w = 2, {{1,0},{1,10}}},
    ['a'] = {w = 5, {{5,6},{5,0}}, {{5,5},{3.5,6},{1.5,6},{0,4.5},{0,1.5},{1.5,0},{3.5,0},{5,1}}},
    ['z'] = {w = 5, {{0,6},{5,6},{0,0},{5,0}}, {{1,9.5},{2.5,7.8},{4,9.5}}},      -- z s kvacicom (ž)
    [' '] = {w = 2},
    ['R'] = {w = 6, {{0,0},{0,10},{4.5,10},{5.5,9},{5.5,6.5},{4.5,5.5},{0,5.5}}, {{3,5.5},{6,0}}},
    ['o'] = {w = 5, {{2.5,6},{4.2,5.2},{5,3},{4.2,0.8},{2.5,0},{0.8,0.8},{0,3},{0.8,5.2},{2.5,6}}},
    ['d'] = {w = 5, {{5,10},{5,0}}, {{5,5},{3.5,6},{1.5,6},{0,4.5},{0,1.5},{1.5,0},{3.5,0},{5,1}}},
    ['i'] = {w = 2, {{1,0},{1,6}}, {{1,8},{1,8.5}}},
    ['c'] = {w = 5, {{5,5},{3.5,6},{1.5,6},{0,4.5},{0,1.5},{1.5,0},{3.5,0},{5,1}}, {{2,7.5},{3.8,9.8}}},  -- c s oksicem (ć)
}
local TEXT = {'B', 'l', 'a', 'z', ' ', 'R', 'o', 'd', 'i', 'c'}        -- z = ž, c = ć
local GAP_LETTER = 2

-- potezi teksta u koordinatama Base sustava; tekst se cita u smjeru -y, "gore" je +x
local function textStrokes()
    local strokes, cx = {}, 0
    for _, ch in ipairs(TEXT) do
        local g = GLYPH[ch]
        for _, st in ipairs(g) do
            local s = {}
            for _, pt in ipairs(st) do s[#s + 1] = {X0 + pt[2] * SCALE, Y0 - (cx + pt[1]) * SCALE} end
            strokes[#strokes + 1] = s
        end
        cx = cx + g.w + GAP_LETTER
    end
    return strokes
end

-- ---- planiranje: popis koraka (jedan okvir = jedan korak simulacije) -------
local function plan(dt)
    local HOME = {X0 + 0.025, 0.0, 0.10}
    local q, ok = ik(targetDown(HOME[1], HOME[2], HOME[3]), Q_SEED)
    if not ok then error('Pocetna poza nije dohvatljiva') end
    local qStart, pos, frames = q, HOME, {}

    local function go(p, speed, pen)
        local path = linePath(q, pos, p, dt, speed)
        for _, st in ipairs(path) do frames[#frames + 1] = {q = st.q, p = st.p, pen = pen} end
        q, pos = path[#path].q, p
    end
    local function gapFrames(g0, g1)
        for k = 1, 6 do frames[#frames + 1] = {gap = g0 + (g1 - g0) * k / 6} end
    end
    local function idle(n) for _ = 1, n do frames[#frames + 1] = {} end end

    frames[#frames + 1] = {valve = true}; idle(VALVE_STEPS)        -- DO0 = 1: hvataljka stisne marker
    frames[#frames + 1] = {check = true}
    gapFrames(GAP_OPEN, GAP_PEN)

    local strokes = textStrokes()
    for _, s in ipairs(strokes) do
        go({s[1][1], s[1][2], Z_UP}, SPEED_MOVE, false)
        go({s[1][1], s[1][2], Z_WRITE}, SPEED_MOVE, true)           -- spusti marker
        for k = 2, #s do go({s[k][1], s[k][2], Z_WRITE}, SPEED_WRITE, true) end
        go({s[#s][1], s[#s][2], Z_UP}, SPEED_MOVE, false)           -- podigni marker
    end
    go(HOME, SPEED_MOVE, false)
    frames[#frames + 1] = {done = true}
    return qStart, frames
end

-- ---- pomocne funkcije za scenu --------------------------------------------
local function toPose(T)
    local R = {{T[1][1], T[1][2], T[1][3]}, {T[2][1], T[2][2], T[2][3]}, {T[3][1], T[3][2], T[3][3]}}
    local tr = R[1][1] + R[2][2] + R[3][3]
    local w, x, y, z, s
    if tr > 0 then
        s = 2 * math.sqrt(tr + 1)
        w, x, y, z = 0.25 * s, (R[3][2] - R[2][3]) / s, (R[1][3] - R[3][1]) / s, (R[2][1] - R[1][2]) / s
    elseif R[1][1] > R[2][2] and R[1][1] > R[3][3] then
        s = 2 * math.sqrt(1 + R[1][1] - R[2][2] - R[3][3])
        w, x, y, z = (R[3][2] - R[2][3]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s, (R[1][3] + R[3][1]) / s
    elseif R[2][2] > R[3][3] then
        s = 2 * math.sqrt(1 + R[2][2] - R[1][1] - R[3][3])
        w, x, y, z = (R[1][3] - R[3][1]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s, (R[2][3] + R[3][2]) / s
    else
        s = 2 * math.sqrt(1 + R[3][3] - R[1][1] - R[2][2])
        w, x, y, z = (R[2][1] - R[1][2]) / s, (R[1][3] + R[3][1]) / s, (R[2][3] + R[3][2]) / s, 0.25 * s
    end
    return {T[1][4], T[2][4], T[3][4], x, y, z, w}
end

local function attach(h, parent, T, name)
    sim.setObjectParent(h, parent, true)
    sim.setObjectPose(h, toPose(T), parent == -1 and sim.handle_world or parent)
    if name then sim.setObjectAlias(h, name) end
    return h
end

local function paint(h, rgb)
    sim.setShapeColor(h, nil, sim.colorcomponent_ambient_diffuse, rgb)
end

local function makeStatic(h)
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
end

local function box(size, T, rgb, parent, name)
    local h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    attach(h, parent, T, name)
    paint(h, rgb)
    makeStatic(h)
    return h
end

local function bar(parent, p0, p1, r, rgb)      -- valjak izmedju tocaka p0 i p1 u sustavu roditelja
    local v = {p1[1] - p0[1], p1[2] - p0[2], p1[3] - p0[3]}
    local L = norm(v)
    if L < 1e-6 then return nil end
    local u = {v[1] / L, v[2] / L, v[3] / L}
    local ax = {-u[2], u[1], 0}                  -- z x u
    local s, c = norm(ax), u[3]
    local R
    if s < 1e-9 then
        R = (c > 0) and {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} or {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}
    else
        local k = {ax[1] / s, ax[2] / s, ax[3] / s}
        local Kx = {{0, -k[3], k[2]}, {k[3], 0, -k[1]}, {-k[2], k[1], 0}}
        R = {}
        for i = 1, 3 do
            R[i] = {}
            for j = 1, 3 do
                local kk = 0
                for m = 1, 3 do kk = kk + Kx[i][m] * Kx[m][j] end
                R[i][j] = (i == j and 1 or 0) + s * Kx[i][j] + (1 - c) * kk
            end
        end
    end
    local T = {{R[1][1], R[1][2], R[1][3], (p0[1] + p1[1]) / 2},
               {R[2][1], R[2][2], R[2][3], (p0[2] + p1[2]) / 2},
               {R[3][1], R[3][2], R[3][3], (p0[3] + p1[3]) / 2},
               {0, 0, 0, 1}}
    local h = sim.createPrimitiveShape(sim.primitiveshape_cylinder, {2 * r, 2 * r, L}, 0)
    attach(h, parent, T)
    paint(h, rgb)
    makeStatic(h)
    return h
end

local function buildScene()
    local world = sim.handle_world
    local GREY, BLUE = {0.65, 0.65, 0.68}, {0.2, 0.45, 0.8}

    box({0.9, 0.6, 0.04}, tf(0.2, 0.0, -0.02), {0.75, 0.6, 0.45}, -1, 'stol')
    box({0.11, 0.40, 0.001}, tf(X0 + 0.025, 0.0, 0.0005), {0.97, 0.97, 0.97}, -1, 'papir')

    -- periferija: zracni kompresor, pripremna jedinica, elektromagnetski ventil
    local tank = sim.createPrimitiveShape(sim.primitiveshape_cylinder, {0.14, 0.14, 0.35}, 0)
    attach(tank, -1, mmul(tf(-0.30, -0.30, 0.07), {{1, 0, 0, 0}, {0, 0, -1, 0}, {0, 1, 0, 0}, {0, 0, 0, 1}}), 'zracni_kompresor')
    paint(tank, {0.8, 0.15, 0.1}); makeStatic(tank)
    box({0.05, 0.05, 0.08}, tf(-0.12, -0.30, 0.04), {0.3, 0.3, 0.3}, -1, 'FRL_pripremna_jedinica')
    box({0.05, 0.03, 0.04}, tf(-0.05, -0.30, 0.02), {0.15, 0.15, 0.15}, -1, 'elektromagnetski_ventil')

    -- robot iz DH tablice
    local base = sim.createDummy(0.05)
    attach(base, world, eye(), 'UR3e_base')
    bar(base, {0, 0, 0}, {0, 0, 0.04}, 0.064, {0.25, 0.25, 0.28})
    -- ploca za logo na bazi robota: dvoklik na plocu > Adjust texture, ucitaj vlastitu sliku logotipa
    box({0.003, 0.06, 0.04}, tf(0.066, 0.0, 0.02), {1, 1, 1}, base, 'logo_ploca')
    local dh0 = sim.createDummy(0.03)
    attach(dh0, base, tf(0, 0, 0, math.pi), 'dh_okvir_0')       -- DH sustav je zarotiran 180 deg oko z

    local mode = sim.jointmode_kinematic or sim.jointmode_passive
    local joints, parent = {}, dh0
    for j = 1, 6 do
        local Tj = (j == 1) and eye() or dh(0, DH_D[j - 1], DH_A[j - 1], DH_AL[j - 1])
        local jh = sim.createJoint(sim.joint_revolute_subtype, mode, 0, {0.05, 0.12})
        attach(jh, parent, Tj, 'zglob_' .. j)
        pcall(sim.setJointInterval, jh, true, {-math.pi, 2 * math.pi})
        bar(jh, {0, 0, -0.035}, {0, 0, 0.035}, 0.04, BLUE)
        bar(jh, {0, 0, 0}, {0, 0, DH_D[j]}, 0.025, GREY)
        bar(jh, {0, 0, DH_D[j]}, {DH_A[j], 0, DH_D[j]}, 0.025, GREY)
        joints[j] = jh
        parent = jh
    end
    local flange = sim.createDummy(0.03)
    attach(flange, joints[6], dh(0, DH_D[6], DH_A[6], DH_AL[6]), 'prirubnica')

    -- SCHUNK PGN-plus-P 50 (pojednostavljeni model) koji drzi marker
    box({0.05, 0.05, 0.06}, tf(0, 0, 0.03), {0.2, 0.2, 0.22}, flange, 'hvataljka_tijelo')
    box({0.012, FINGER_T, 0.06}, tf(0, (GAP_OPEN + FINGER_T) / 2, 0.09), {0.85, 0.85, 0.85}, flange, 'prst_1')
    box({0.012, FINGER_T, 0.06}, tf(0, -(GAP_OPEN + FINGER_T) / 2, 0.09), {0.85, 0.85, 0.85}, flange, 'prst_2')
    bar(flange, {0, 0, 0.06}, {0, 0, TOOL_Z}, 0.004, {0.05, 0.1, 0.8})   -- marker
    local tcp = sim.createDummy(0.02)
    attach(tcp, flange, tf(0, 0, TOOL_Z), 'TCP')

    -- pocetna poza (iznad papira)
    local qs = ik(targetDown(X0 + 0.025, 0.0, 0.10), Q_SEED)
    for i = 1, 6 do sim.setJointPosition(joints[i], qs[i]) end
end

local NAMES = {'UR3e_base', 'zglob_1', 'zglob_2', 'zglob_3', 'zglob_4', 'zglob_5', 'zglob_6',
               'prirubnica', 'logo_ploca', 'papir', 'prst_1', 'prst_2', 'TCP', 'elektromagnetski_ventil'}

local LEGACY = {'UR3e_base', 'stol', 'papir', 'zracni_kompresor', 'FRL_pripremna_jedinica',
                'elektromagnetski_ventil', 'mjesto_A', 'mjesto_B', 'kocka_1', 'kocka_2', 'kocka_3'}

local function clearOld()             -- brise stare dijelove celije (npr. iz starije verzije skripte)
    local all, byAlias = sim.getObjectsInTree(sim.handle_scene), {}
    for _, h in ipairs(all) do
        local a = sim.getObjectAlias(h)
        byAlias[a] = byAlias[a] or {}
        table.insert(byAlias[a], h)
    end
    local del = {}
    for _, n in ipairs(LEGACY) do
        for _, h in ipairs(byAlias[n] or {}) do
            for _, c in ipairs(sim.getObjectsInTree(h)) do del[c] = true end
        end
    end
    local list = {}
    for h in pairs(del) do list[#list + 1] = h end
    if #list > 0 then pcall(sim.removeObjects, list) end
end

local function findScene()            -- trazi vec izgradjene objekte po imenu (ako je scena spremljena)
    local byName = {}
    for _, h in ipairs(sim.getObjectsInTree(sim.handle_scene)) do byName[sim.getObjectAlias(h)] = h end
    for _, n in ipairs(NAMES) do if not byName[n] then return nil end end
    local S = {joints = {}, tcp = byName.TCP, flange = byName.prirubnica, f1 = byName.prst_1, f2 = byName.prst_2,
               valve = byName.elektromagnetski_ventil, base = byName.UR3e_base}
    for i = 1, 6 do S.joints[i] = byName['zglob_' .. i] end
    return S
end

-- ---- izvodjenje -------------------------------------------------------------
local S, frames, fi, finished, ink, lastP, lastPen

local function clearInk()          -- papir se ocisti: ukloni stari trag
    if ink then pcall(sim.removeDrawingObject, ink); ink = nil end
end

local function setQ(q)
    for i = 1, 6 do sim.setJointPosition(S.joints[i], q[i]) end
end

local function setFingers(gap)
    local off = gap / 2 + FINGER_T / 2
    sim.setObjectPosition(S.f1, {0, off, 0.09}, S.flange)
    sim.setObjectPosition(S.f2, {0, -off, 0.09}, S.flange)
end

local function setValve(on)
    sim.setInt32Signal('DO0', on and 1 or 0)               -- digitalni izlaz kontrolera -> ventil
    paint(S.valve, on and {0.1, 0.8, 0.2} or {0.15, 0.15, 0.15})
end

function sysCall_init()                                    -- pri otvaranju scene: trajna izgradnja
    S = findScene()
    if not S then
        clearOld()
        buildScene()
        S = findScene()
        print('Celija izgradjena. Spremi scenu (File > Save scene) pa robot ostaje u njoj.')
    end
end

function sysCall_beforeSimulation()                        -- Play: planiranje i priprema
    local dt = sim.getSimulationTimeStep()
    local qStart
    qStart, frames = plan(dt)
    setQ(qStart); setFingers(GAP_OPEN); setValve(false)
    sim.setFloatSignal('p_bar', P_TANK)                    -- kompresor + pripremna jedinica
    clearInk()
    ink = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 200000, INK)
    lastP, lastPen = nil, false
    local p = sim.getObjectPosition(S.tcp, S.base)         -- provjera: model u sceni = DH proracun
    local ref = fk(qStart)
    local err = 1000 * math.sqrt((p[1] + ref[1][4]) ^ 2 + (p[2] + ref[2][4]) ^ 2 + (p[3] - ref[3][4]) ^ 2)
    print(string.format('Provjera modela: TCP odstupa %.2f mm od DH proracuna', err))
    if err > 1.0 then print('UPOZORENJE: model u sceni ne odgovara DH tablici') end
    print(string.format('Plan: %d koraka (oko %.1f s simulacije), tekst: %s', #frames, #frames * dt, TEXT_NAME))
    fi, finished = 1, false
end

function sysCall_actuation()
    if sim.getSimulationState() == sim.simulation_stopped or finished or not frames then return end
    local f = frames[fi]
    if not f then finished = true; return end
    fi = fi + 1
    if f.q then setQ(f.q) end
    if f.gap then setFingers(f.gap) end
    if f.valve ~= nil then setValve(f.valve) end
    if f.check and sim.getFloatSignal('p_bar') < P_MIN then
        finished = true
        print('GRESKA: tlak zraka ispod minimalnog, hvataljka ne drzi marker.')
        return
    end
    if f.p then
        local cur = {f.p[1], f.p[2], Z_WRITE - 0.0005}
        if f.pen and lastPen and lastP then
            sim.addDrawingObjectItem(ink, {lastP[1], lastP[2], lastP[3], cur[1], cur[2], cur[3]})
        end
        lastP, lastPen = cur, f.pen
    end
    if f.done then
        finished = true
        print('Gotovo: ispisano "Blaž Rodić". Zaustavi simulaciju tipkom Stop.')
    end
end

function sysCall_afterSimulation()
    clearInk()
    pcall(function() setValve(false) end)                  -- ventil u pocetno stanje
end

function sysCall_cleanup()
    clearInk()
end
