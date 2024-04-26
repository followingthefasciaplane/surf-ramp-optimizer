#include "surf_ramp_optimizer.h"
#include <immintrin.h>
#include <limits>
#include <igameevents.h>
#include <algorithm>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

// Forward declarations
class SurfRampOptimizer;
class CBrachistochroneOptimizer;
class PlayerMovementTracking;
class BVHNode;
class Matrix;
class Vector;
class Ramp;
class CBaseEntity;
namespace CollisionDetection;
namespace NumericalMethods;  
namespace MathUtils;
namespace Optimization;
namespace Pathfinding;

// Global constants
constexpr float MAX_PLAYER_DIST_TO_PATH = 50.0f;
constexpr float GRADIENT_TOLERANCE = 1e-6f;
constexpr int MAX_OPTIMIZATION_ITERATIONS = 100;

// Global variables
SurfRampOptimizer g_SurfRampOptimizer;
SMEXT_LINK(&g_SurfRampOptimizer);

IForward* g_fwdOnStartTouchRamp = nullptr;

IGameConfig* g_pGameConf = nullptr;
ISDKTools* g_pSDKTools = nullptr;
IServerGameEnts* gameents = nullptr;
IGameHelpers* g_pGameHelpers = nullptr;
CGlobalVars* gpGlobals = nullptr;

IBinTools* g_pBinTools = nullptr;
IGameMovement* g_pGameMovement = nullptr;
IPlayerInfoManager* playerinfomngr = nullptr;

std::mutex g_SurfRampsMutex;
std::mutex g_RampCacheMutex;
std::vector<std::unique_ptr<CBrachistochroneOptimizer>> g_SurfRamps;
BVHNode* g_BVHRoot = nullptr;
std::unordered_map<CBaseEntity*, Ramp> g_RampCache;

// SurfRampOptimizer methods

bool SurfRampOptimizer::SDK_OnLoad(char* error, size_t maxlength, bool late) 
{
    if (!SetupGameInterfaces(error, maxlength)) 
    {
        return false;
    }

    if (!LoadConfigurations(error, maxlength)) 
    {
        return false;
    }

    InitializeDataStructures();

    return true;
}

bool SurfRampOptimizer::SetupGameInterfaces(char* error, size_t maxlength) 
{
    gameents = gamehelpers->GetIServerGameEnts();
    g_pGameHelpers = gamehelpers;
    gpGlobals = gamehelpers->GetGlobalVars();

    g_pSDKTools = sdktools;
    g_pBinTools = g_pSDKTools->GetBinTools();
    g_pGameMovement = g_pBinTools->GetGameMovement();
    playerinfomngr = playerhelpers->GetPlayerInfoManager();

    if (!gameents || !g_pGameHelpers || !gpGlobals || !g_pSDKTools || !g_pBinTools || !g_pGameMovement || !playerinfomngr) 
    {
        snprintf(error, maxlength, "Failed to get required game interfaces");
        return false;
    }

    return true;
}

bool SurfRampOptimizer::LoadConfigurations(char* error, size_t maxlength) 
{
    sharesys->AddDependency(myself, "bintools.ext", true, true);
    sharesys->AddDependency(myself, "sdktools.ext", true, true);
    sharesys->AddDependency(myself, "playerhelpers.ext", true, true);

    sharesys->RegisterLibrary(myself, "surframpoptimizer");

    return true;
}

void SurfRampOptimizer::InitializeDataStructures() 
{
    g_SurfRamps.clear();
    delete g_BVHRoot;
    g_BVHRoot = new BVHNode(Vector(-1000.0f, -1000.0f, -1000.0f), Vector(1000.0f, 1000.0f, 1000.0f));
}

void SurfRampOptimizer::SDK_OnUnload() 
{
    g_pSDKTools->RemoveEntityListener(&g_SurfRampOptimizer);

    {
        std::lock_guard<std::mutex> lock(g_SurfRampsMutex);
        g_SurfRamps.clear();
    }

    delete g_BVHRoot;
    g_BVHRoot = nullptr;
}

void SurfRampOptimizer::SDK_OnAllLoaded() 
{
    SM_GET_LATE_IFACE(SDKTOOLS, g_pSDKTools);
    SM_GET_LATE_IFACE(BINTOOLS, g_pBinTools);

    if (!g_pSDKTools || !g_pBinTools)
    {
        smutils->LogError(myself, "Failed to get bin tools or SDK tools interfaces");
        return;
    }

    g_pGameMovement = g_pBinTools->GetGameMovement();

    if (!g_pGameMovement)
    {
        smutils->LogError(myself, "Failed to get IGameMovement interface");
        return;
    }

    g_fwdOnStartTouchRamp = g_pGameHelpers->CreateForward("OnStartTouchRamp", ET_Event, 2, nullptr);

    g_pSDKTools->AddEntityListener(&g_SurfRampOptimizer);

    CollisionDetection::Initialize();

    smutils->LogMessage(myself, "SurfRampOptimizer is loaded (Server Tickrate: %.2f)", gpGlobals->tickRate);
}

bool SurfRampOptimizer::QueryRunning(char* error, size_t maxlength) 
{
    SM_CHECK_IFACE(SDKTOOLS, g_pSDKTools);
    SM_CHECK_IFACE(BINTOOLS, g_pBinTools);
    return true;
}

void SurfRampOptimizer::OnEntityCreated(CBaseEntity* pEntity, const char* classname) 
{
    if (!pEntity || !classname || std::string(classname) != "surf_ramp")
    {
        return;
    }

    Ramp ramp = GetRampFromEntity(pEntity);
    auto optimizer = std::make_unique<CBrachistochroneOptimizer>(ramp, pEntity);

    {
        std::lock_guard<std::mutex> lock(g_SurfRampsMutex);
        pEntity->SetTouch(SurfRampOptimizer::OnStartTouch);
        g_SurfRamps.push_back(std::move(optimizer));
    }

    CollisionDetection::InsertRamp(&ramp);
}

void SurfRampOptimizer::OnEntityDestroyed(CBaseEntity* pEntity) 
{
    std::lock_guard<std::mutex> lock(g_SurfRampsMutex);
    g_SurfRamps.erase(std::remove_if(g_SurfRamps.begin(), g_SurfRamps.end(), [&](const auto& optimizer) 
    {
        return optimizer->GetRamp().startPoint == pEntity->GetAbsOrigin();
    }), g_SurfRamps.end());

    {
        std::lock_guard<std::mutex> lock(g_RampCacheMutex);
        g_RampCache.erase(pEntity);
    }
}

void SurfRampOptimizer::OnStartTouch(CBaseEntity* pOther) 
{
    if (!pOther || !pOther->IsPlayer())
    {
        return;
    }

    if (pOther->GetClassName() == "surf_ramp") 
    {
        FireOnStartTouchRampForward(pOther, gameents->BaseEntityToEdict(this));

        Ramp ramp;
        {
            std::lock_guard<std::mutex> lock(g_RampCacheMutex);
            auto it = g_RampCache.find(this);
            if (it != g_RampCache.end()) 
            {
                ramp = it->second;
            }
        }

        if (ramp.startPoint.Length() > 0)
        {
            CBasePlayer* pPlayer = static_cast<CBasePlayer*>(pOther);
            if (pPlayer)
            {
                Optimize(pPlayer, ramp);
            }
        }

        for (const auto& bc : g_SurfRamps) 
        {
            if (bc->GetRamp() == ramp) 
            {
                Vector pos = pOther->GetAbsOrigin();
                Vector vel = pOther->GetAbsVelocity();
                float tickInterval = 1.0f / gpGlobals->tickRate;
                
                std::vector<Vector> points = bc->Optimize(pos, vel, tickInterval);
                bc->Update(pos, vel, pOther->GetAbsVelocity() - vel, tickInterval);
                break;
            }
        }
    }
}

bool SurfRampOptimizer::SDK_OnMetamodLoad(ISmmAPI* ismm, char* error, size_t maxlen, bool late) 
{
    GET_V_IFACE_CURRENT(GetEngineFactory, g_pCVar, ICvar, CVAR_INTERFACE_VERSION);
    GET_V_IFACE_CURRENT(GetEngineFactory, gameents, IServerGameEnts, INTERFACEVERSION_SERVERGAMEENTS);
    GET_V_IFACE_ANY(GetServerFactory, playerhelpers, IPlayerInfoManager, INTERFACEVERSION_PLAYERINFOMANAGER);

    return true;
}

// CBrachistochroneOptimizer methods

CBrachistochroneOptimizer::CBrachistochroneOptimizer(const Ramp& ramp, CBaseEntity* player)
    : m_Ramp(ramp)
    , m_Player(player)
    , m_PlayerTracker(MAX_PLAYER_DIST_TO_PATH)
{
    m_AirAccelerate = gpGlobals->airAccelerate;
    m_Gravity = gpGlobals->gravity;
}

std::vector<Vector> CBrachistochroneOptimizer::Optimize(const Vector& startPos, const Vector& startVel, float tickInterval) 
{
    float pathTime = NumericalMethods::CalculateBrachistochronePathTime(startPos, m_Ramp.endPoint, m_Gravity);

    if (pathTime <= 0.0f)
    {
        return {}; 
    }

    const float timeStep = tickInterval;

    Vector prevPos = startPos;
    Vector currVel = startVel;
    Vector currAccel(0.0f, 0.0f, -m_Gravity);

    m_OptimizedPath.clear();
    m_OptimizedPath.push_back(startPos);

    for (float t = 0.0f; t <= pathTime; t += timeStep) 
    {
        Vector pos = NumericalMethods::SolveBrachistochrone(startPos, m_Ramp.endPoint, t, m_Gravity, m_AirAccelerate);
        Vector accelDir = (pos - prevPos).Normalized();

        float wishSpeed = currVel.Length();
        float maxWishSpeed = g_pCVar->FindVar("sv_air_max_wishspeed")->GetFloat();
        float maxVelocity = g_pCVar->FindVar("sv_maxvelocity")->GetFloat();
        float accelAmount = m_AirAccelerate * timeStep * std::min(wishSpeed, maxWishSpeed);

        currVel += accelDir * accelAmount - currAccel * timeStep;
        currAccel = accelDir * accelAmount / timeStep;

        if (currVel.Length() > maxVelocity)
        {
            currVel = currVel.Normalized() * maxVelocity;
        }

        if (IsPathValid(pos))
        {
            m_OptimizedPath.push_back(pos);
            prevPos = pos;
        }
        else
        {
            break;
        }
    }

    if (IsPathValid(m_Ramp.endPoint))
    {
        m_OptimizedPath.push_back(m_Ramp.endPoint);
    }

    m_OptimizedPath = Optimization::OptimizePath(m_OptimizedPath, m_Ramp, m_Gravity, m_AirAccelerate);

    return m_OptimizedPath;
}

void CBrachistochroneOptimizer::Update(const Vector& playerPos, const Vector& playerVel, const Vector& playerAccel, float tickInterval)
{
    m_PlayerTracker.UpdatePlayerState(playerPos, playerVel, playerAccel, tickInterval);

    std::vector<Ramp> ramps;
    {
        std::lock_guard<std::mutex> lock(g_RampCacheMutex);
        for (auto it = g_RampCache.begin(); it != g_RampCache.end(); ++it) 
        {
            ramps.push_back(it->second);
        }
    }

    m_PlayerTracker.RecalibratePath(m_OptimizedPath, ramps, *this, tickInterval);
}

bool CBrachistochroneOptimizer::IsPathValid(const Vector& point) const 
{
    return !CollisionDetection::Intersects(point, m_Ramp);
}

const Ramp& CBrachistochroneOptimizer::GetRamp() const 
{
    return m_Ramp;
}

const std::vector<Vector>& CBrachistochroneOptimizer::GetPath() const 
{
    return m_OptimizedPath;
}

CBaseEntity* CBrachistochroneOptimizer::GetPlayer() const 
{
    return m_Player;
}

// PlayerMovementTracking methods

PlayerMovementTracking::PlayerMovementTracking(float maxDeviationDistance, size_t bufferCapacity)
    : m_MaxDeviationDistance(maxDeviationDistance)
    , m_Positions(bufferCapacity)
    , m_Velocities(bufferCapacity)
    , m_Accelerations(bufferCapacity)
    , m_TimeIntervals(bufferCapacity) 
{}

void PlayerMovementTracking::UpdatePlayerState(const Vector& position, const Vector& velocity, const Vector& acceleration, float tickInterval) 
{
    m_Positions.push_back(position);
    m_Velocities.push_back(velocity);
    m_Accelerations.push_back(acceleration);
    m_TimeIntervals.push_back(tickInterval);

    if (m_Positions.size() > m_Positions.capacity()) 
    {
        m_Positions.pop_front();
        m_Velocities.pop_front();
        m_Accelerations.pop_front();
        m_TimeIntervals.pop_front();
    }
}

Vector PlayerMovementTracking::EstimatePlayerPosition(float timeStep) const 
{
    if (m_Positions.empty())
    {
        return {};
    }

    size_t n = m_Positions.size() - 1;
    Vector position = m_Positions[n];
    Vector velocity = m_Velocities[n];
    Vector acceleration = m_Accelerations[n];
    float timeInterval = m_TimeIntervals[n];

    return position + velocity * timeStep + 0.5f * acceleration * timeStep * timeStep;
}

bool PlayerMovementTracking::HasDeviatedFromPath(const std::vector<Vector>& path) const 
{
    if (m_Positions.empty() || path.empty())
    {
        return false;
    }

    Vector playerPos = m_Positions.back();
    float minDistance = std::numeric_limits<float>::max(); /////fix

    for (const Vector& point : path)
    {
        float distance = (playerPos - point).LengthSqr();
        minDistance = std::min(minDistance, distance);
    }

    return minDistance > m_MaxDeviationDistance * m_MaxDeviationDistance;
}

void PlayerMovementTracking::RecalibratePath(const std::vector<Vector>& currentPath, const std::vector<Ramp>& ramps, CBrachistochroneOptimizer& optimizer, float tickInterval)
{
    if (HasDeviatedFromPath(currentPath))
    {
        Vector currentPos = m_Positions.back();
        Vector currentVel = m_Velocities.back();
        Vector goal = currentPath.back();

        std::vector<Vector> newPath = Pathfinding::Pathfinding(currentPos, goal, ramps);

        if (!newPath.empty())
        {
            optimizer.m_OptimizedPath = optimizer.Optimize(currentPos, currentVel, tickInterval);
        }
    }
}

// BVHNode methods

BVHNode::BVHNode(const Vector& min, const Vector& max)
    : minExtents(min)
    , maxExtents(max)
    , left(nullptr)
    , right(nullptr)
{}

BVHNode::~BVHNode()
{
    delete left;
    delete right;
}

void BVHNode::Insert(Ramp* ramp)
{
    if (!left && !right)
    {
        objects.push_back(ramp);

        if (objects.size() > 4)
        {
            Vector center = (minExtents + maxExtents) * 0.5f;
            left = new BVHNode(minExtents, center);
            right = new BVHNode(center, maxExtents);

            for (Ramp* obj : objects)
            {
                if (obj->minExtents.x < center.x)
                {
                    left->Insert(obj);
                }
                else
                {
                    right->Insert(obj);
                }
            }

            objects.clear();
        }
    }
    else
    {
        if (ramp->minExtents.x < left->maxExtents.x)
        {
            left->Insert(ramp);
        }
        else
        {
            right->Insert(ramp);
        }
    }
}

std::vector<Ramp*> BVHNode::Query(const Vector& point) const
{
    std::vector<Ramp*> result;

    if (point.x >= minExtents.x && point.x <= maxExtents.x &&
        point.y >= minExtents.y && point.y <= maxExtents.y &&
        point.z >= minExtents.z && point.z <= maxExtents.z)
    {
        result.insert(result.end(), objects.begin(), objects.end());

        if (left)
        {
            std::vector<Ramp*> leftResult = left->Query(point);
            result.insert(result.end(), leftResult.begin(), leftResult.end());
        }

        if (right)
        {
            std::vector<Ramp*> rightResult = right->Query(point);
            result.insert(result.end(), rightResult.begin(), rightResult.end());
        }
    }

    return result;
}

// CollisionDetection methods

void CollisionDetection::Initialize() 
{
    g_BVHRoot = new BVHNode(Vector(-1000.0f, -1000.0f, -1000.0f), Vector(1000.0f, 1000.0f, 1000.0f));
}

void CollisionDetection::Shutdown() 
{
    delete g_BVHRoot;
    g_BVHRoot = nullptr;
}

void CollisionDetection::InsertRamp(Ramp* ramp)
{
    g_BVHRoot->Insert(ramp);
}

bool CollisionDetection::Intersects(const Vector& point, const Ramp& ramp)
{
    Ramp pointRamp;
    pointRamp.startPoint = point;
    pointRamp.endPoint = point;
    pointRamp.width = 0.0f;

    return GJKIntersection(pointRamp, ramp);
}

bool CollisionDetection::GJKIntersection(const Ramp& ramp1, const Ramp& ramp2)
{
    Vector support[4];
    Vector direction = ramp2.startPoint - ramp1.startPoint;

    support[0] = SupportPoint(ramp1, ramp2, direction);
    support[1] = SupportPoint(ramp1, ramp2, -direction);

    Vector simplex[3];
    simplex[0] = support[0];
    simplex[1] = support[1];

    direction = MathUtils::SIMDCrossProduct(simplex[1] - simplex[0], -simplex[0]);

    int index = 2;
    while (true)
    {
        support[index] = SupportPoint(ramp1, ramp2, direction);

        if (MathUtils::SIMDDotProduct(support[index], direction) < 0)
        {
            return false;
        }

        simplex[index] = support[index];

        if (UpdateSimplex(simplex, direction, index))
        {
            return true;
        }

        index = (index + 1) % 3;
    }
}

Vector CollisionDetection::SupportPoint(const Ramp& ramp1, const Ramp& ramp2, const Vector& direction)
{
    Vector support1 = FarthestPointInDirection(ramp1, direction);
    Vector support2 = FarthestPointInDirection(ramp2, -direction);
    return support1 - support2;
}

Vector CollisionDetection::FarthestPointInDirection(const Ramp& ramp, const Vector& direction)
{
    float maxDot = -std::numeric_limits<float>::max();
    Vector farthestPoint;

    std::vector<Vector> vertices =
    {
        ramp.startPoint,
        ramp.endPoint,
        ramp.startPoint + Vector(0, ramp.width, 0),
        ramp.endPoint + Vector(0, ramp.width, 0)
    };

    for (const Vector& vertex : vertices)
    {
        float dot = MathUtils::SIMDDotProduct(vertex, direction);
        if (dot > maxDot)
        {
            maxDot = dot;
            farthestPoint = vertex;
        }
    }

    return farthestPoint;
}

bool CollisionDetection::UpdateSimplex(Vector* simplex, Vector& direction, int index)
{
    Vector a = simplex[index];
    Vector b = simplex[(index + 1) % 3];
    Vector c = simplex[(index + 2) % 3];

    Vector ab = b - a;
    Vector ac = c - a;
    Vector ao = -a;

    Vector abPerp = MathUtils::SIMDCrossProduct(ac, ab);

    if (MathUtils::SIMDDotProduct(abPerp, ao) > 0)
    {
        direction = abPerp;
    }
    else
    {
        Vector acPerp = MathUtils::SIMDCrossProduct(ab, ac);

        if (MathUtils::SIMDDotProduct(acPerp, ao) > 0)
        {
            simplex[0] = a;
            simplex[1] = c;
            direction = acPerp;
        }
        else
        {
            if (MathUtils::SIMDDotProduct(ab, ao) > 0)
            {
                simplex[0] = a;
                simplex[1] = b;
                direction = MathUtils::SIMDCrossProduct(ab, ao);
            }
            else
            {
                simplex[0] = b;
                simplex[1] = c;
                direction = MathUtils::SIMDCrossProduct(ac, ao);
            }
        }
    }

    return false;
}

// NumericalMethods methods

Vector NumericalMethods::SolveBrachistochrone(const Vector& startPos, const Vector& endPos, float time, float gravity, float airAccelerate)
{
    return SolveBrachistochroneAdaptive(startPos, endPos, time, gravity, airAccelerate, 1e-6f);
}

Vector NumericalMethods::SolveBrachistochroneAdaptive(const Vector& startPos, const Vector& endPos, float time, float gravity, float airAccelerate, float errorTolerance)
{
    auto brachistochroneEquation = [](float t, Vector y, void* params)
    {
        Vector* p = static_cast<Vector*>(params);
        Vector startPos = p[0];
        Vector endPos = p[1];
        float gravity = p[2].x;
        float airAccelerate = p[3].x;

        Vector pos = Vector(y.x, y.y, y.z);
        Vector vel = Vector(y.v, y.w, y.u);
        Vector accel = Vector(0.0f, 0.0f, -gravity) + airAccelerate * vel.Normalized();

        return Vector(vel.x, vel.y, vel.z, accel.x, accel.y, accel.z);
    };

    Vector y0 = Vector(startPos.x, startPos.y, startPos.z, 0.0f, 0.0f, 0.0f);
    Vector params[] = { startPos, endPos, Vector(gravity, 0.0f, 0.0f), Vector(airAccelerate, 0.0f, 0.0f) };

    float h = 1e-3f;
    Vector result = DormandPrince54(0.0f, time, y0, brachistochroneEquation, params, h);

    return Vector(result.x, result.y, result.z);
}

float NumericalMethods::CalculateBrachistochronePathTime(const Vector& startPos, const Vector& endPos, float gravity)
{
    auto integrand = [](float t, void* params)
    {
        Vector* p = static_cast<Vector*>(params);
        Vector startPos = p[0];
        Vector endPos = p[1];
        float gravity = p[2].x;

        Vector displacement = endPos - startPos;
        float distanceSquared = displacement.LengthSqr();
        float height = std::abs(displacement.z);

        return std::sqrt(distanceSquared / (2.0f * gravity * height));
    };

    Vector params[] = { startPos, endPos, Vector(gravity, 0.0f, 0.0f) };

    int n = 100;
    return GaussianQuadrature(0.0f, 1.0f, n, integrand, params);
}

Vector NumericalMethods::CalculateGradient(const Vector& point, const Ramp& ramp, float gravity, float airAccelerate)
{
    float h = 0.001f;

    Vector dx = SolveBrachistochrone(point + Vector(h, 0.0f, 0.0f), ramp.endPoint, gravity, airAccelerate) -
                SolveBrachistochrone(point, ramp.endPoint, gravity, airAccelerate);
    Vector dy = SolveBrachistochrone(point + Vector(0.0f, h, 0.0f), ramp.endPoint, gravity, airAccelerate) -
                SolveBrachistochrone(point, ramp.endPoint, gravity, airAccelerate);
    Vector dz = SolveBrachistochrone(point + Vector(0.0f, 0.0f, h), ramp.endPoint, gravity, airAccelerate) -
                SolveBrachistochrone(point, ramp.endPoint, gravity, airAccelerate);

    return Vector(dx.x / h, dy.y / h, dz.z / h);
}

float NumericalMethods::GaussianQuadrature(float a, float b, int n, float (*f)(float, void*), void* params)
{
    const float x[] = { -0.5773502692, 0.5773502692 };
    const float w[] = { 1.0, 1.0 };

    float h = (b - a) / n;
    float sum = 0.0;

    for (int i = 0; i < n; ++i)
    {
        float x0 = a + i * h;
        float x1 = a + (i + 1) * h;
        float mid = (x0 + x1) / 2;
        float dx = (x1 - x0) / 2;

        for (int j = 0; j < 2; ++j)
        {
            float xj = mid + dx * x[j];
            sum += w[j] * f(xj, params);
        }
    }

    return sum * h / 2;
}

Vector NumericalMethods::DormandPrince54(float t0, float tf, const Vector& y0, float (*f)(float, Vector, void*), void* params, float& h)
{
    const float a[] = { 0.0, 0.2, 0.3, 0.8, 8.0 / 9.0, 1.0, 1.0 };
    const float b[][6] =
    {
        {0.2},
        {3.0 / 40.0, 9.0 / 40.0},
        {44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0},
        {19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0},
        {9017.0 / 3168.0, -355.0 / 33.0, 46732.0 / 5247.0, 49.0 / 176.0, -5103.0 / 18656.0},
        {35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0}
    };
    const float c[] = { 35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0, 0.0 };
    const float d[] = { 5179.0 / 57600.0, 0.0, 7571.0 / 16695.0, 393.0 / 640.0, -92097.0 / 339200.0, 187.0 / 2100.0, 1.0 / 40.0 };

    Vector y = y0;
    float t = t0;
    float h_min = 1e-6;
    float h_max = tf - t0;
    float tol = 1e-6;

    while (t < tf)
    {
        if (t + h > tf)
        {
            h = tf - t;
        }

        Vector k1 = f(t, y, params);
        Vector k2 = f(t + a[1] * h, y + h * (b[1][0] * k1), params);
        Vector k3 = f(t + a[2] * h, y + h * (b[2][0] * k1 + b[2][1] * k2), params);
        Vector k4 = f(t + a[3] * h, y + h * (b[3][0] * k1 + b[3][1] * k2 + b[3][2] * k3), params);
        Vector k5 = f(t + a[4] * h, y + h * (b[4][0] * k1 + b[4][1] * k2 + b[4][2] * k3 + b[4][3] * k4), params);
        Vector k6 = f(t + a[5] * h, y + h * (b[5][0] * k1 + b[5][1] * k2 + b[5][2] * k3 + b[5][3] * k4 + b[5][4] * k5), params);

        Vector y_next = y + h * (c[0] * k1 + c[1] * k2 + c[2] * k3 + c[3] * k4 + c[4] * k5 + c[5] * k6);
        Vector y_error = h * (d[0] * k1 + d[1] * k2 + d[2] * k3 + d[3] * k4 + d[4] * k5 + d[5] * k6 + d[6] * f(t + h, y_next, params));

        float error = y_error.Length();
        float h_new = h * std::pow(tol / error, 0.2);
        h_new = std::max(h_min, std::min(h_new, h_max));

        if (error < tol)
        {
            t += h;
            y = y_next;
        }

        h = h_new;
    }

    return y;
}

// MathUtils methods

Vector MathUtils::SIMDCrossProduct(const Vector& a, const Vector& b)
{
    __m128 a_simd = _mm_set_ps(0.0f, a.z, a.y, a.x);
    __m128 b_simd = _mm_set_ps(0.0f, b.z, b.y, b.x);

    __m128 a_yzx = _mm_shuffle_ps(a_simd, a_simd, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 b_yzx = _mm_shuffle_ps(b_simd, b_simd, _MM_SHUFFLE(3, 0, 2, 1));

    __m128 c_zxy = _mm_sub_ps(_mm_mul_ps(a_simd, b_yzx), _mm_mul_ps(a_yzx, b_simd));

    return Vector(c_zxy[0], c_zxy[1], c_zxy[2]);
}

float MathUtils::SIMDDotProduct(const Vector& a, const Vector& b)
{
    __m128 a_simd = _mm_set_ps(0.0f, a.z, a.y, a.x);
    __m128 b_simd = _mm_set_ps(0.0f, b.z, b.y, b.x);

    __m128 result = _mm_mul_ps(a_simd, b_simd);
    result = _mm_hadd_ps(result, result);
    result = _mm_hadd_ps(result, result);

    return _mm_cvtss_f32(result);
}

float MathUtils::SIMDLength(const Vector& v)
{
    __m128 v_simd = _mm_set_ps(0.0f, v.z, v.y, v.x);
    __m128 squared = _mm_mul_ps(v_simd, v_simd);
    squared = _mm_hadd_ps(squared, squared);
    squared = _mm_hadd_ps(squared, squared);

    return _mm_cvtss_f32(_mm_sqrt_ss(squared));
}

Vector MathUtils::SIMDNormalized(const Vector& v)
{
    __m128 v_simd = _mm_set_ps(0.0f, v.z, v.y, v.x);
    __m128 squared = _mm_mul_ps(v_simd, v_simd);
    squared = _mm_hadd_ps(squared, squared);
    squared = _mm_hadd_ps(squared, squared);

    __m128 length = _mm_sqrt_ss(squared);
    __m128 normalized = _mm_div_ps(v_simd, _mm_shuffle_ps(length, length, 0));

    return Vector(normalized[0], normalized[1], normalized[2]);
}

// Optimization methods

std::vector<Vector> Optimization::OptimizePath(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate)
{
    if (path.empty())
    {
        return {};
    }
    return OptimizePathBFGS(path, ramp, gravity, airAccelerate);
}

std::vector<Vector> Optimization::OptimizePathConjugateGradient(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate)
{
    if (path.empty())
    {
        return {};
    }
    
    std::vector<Vector> optimizedPath = path;
    std::vector<Vector> gradients(path.size());
    Vector direction(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < MAX_OPTIMIZATION_ITERATIONS; ++i)
    {
        bool converged = true;

        for (size_t j = 1; j < optimizedPath.size() - 1; ++j)
        {
            Vector& point = optimizedPath[j];
            gradients[j] = NumericalMethods::CalculateGradient(point, ramp, gravity, airAccelerate);

            if (gradients[j].LengthSqr() > GRADIENT_TOLERANCE)
            {
                converged = false;
                if (i == 0)
                {
                    direction = -gradients[j];
                }
                else
                {
                    float beta = std::max(0.0f, gradients[j].Dot(gradients[j] - gradients[j - 1]) / gradients[j - 1].Dot(gradients[j - 1]));
                    direction = -gradients[j] + beta * direction;
                }

                float alpha = LineSearch(point, direction, ramp, gravity, airAccelerate);
                point -= direction * alpha;
            }
        }

        if (converged)
        {
            break;
        }
    }

    return optimizedPath;
}

std::vector<Vector> Optimization::OptimizePathBFGS(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate)
{
    std::vector<Vector> optimizedPath = path;
    std::vector<Vector> gradients(path.size());
    std::vector<Matrix> inverseHessians(path.size(), Matrix::Identity(3));

    for (int i = 0; i < MAX_OPTIMIZATION_ITERATIONS; ++i)
    {
        bool converged = true;

        for (size_t j = 1; j < optimizedPath.size() - 1; ++j)
        {
            Vector& point = optimizedPath[j];
            gradients[j] = NumericalMethods::CalculateGradient(point, ramp, gravity, airAccelerate);

            if (gradients[j].LengthSqr() > GRADIENT_TOLERANCE)
            {
                converged = false;
                Vector direction = -inverseHessians[j] * gradients[j];

                float alpha = LineSearch(point, direction, ramp, gravity, airAccelerate).first;
                point += direction * alpha;

                Vector gradientDifference = NumericalMethods::CalculateGradient(point, ramp, gravity, airAccelerate) - gradients[j];
                UpdateInverseHessian(inverseHessians[j], direction * alpha, gradientDifference);
            }
        }

        if (converged)
        {
            break;
        }
    }

    return optimizedPath;
}

bool Optimization::SatisfiesWolfeConditions(const Vector& point, const Vector& direction, float alpha, const Ramp& ramp, float gravity, float airAccelerate, float c1, float c2)
{
    Vector newPoint = point - direction * alpha;
    float newCost = CalculatePathCost(newPoint, ramp, gravity, airAccelerate);
    float oldCost = CalculatePathCost(point, ramp, gravity, airAccelerate);
    Vector gradient = NumericalMethods::CalculateGradient(point, ramp, gravity, airAccelerate);

    if (newCost <= oldCost + c1 * alpha * gradient.Dot(direction))
    {
        Vector newGradient = NumericalMethods::CalculateGradient(newPoint, ramp, gravity, airAccelerate);
        if (std::abs(newGradient.Dot(direction)) <= c2 * std::abs(gradient.Dot(direction)))
        {
            return true;
        }
    }

    return false;
}

std::pair<float, bool> Optimization::LineSearch(const Vector& point, const Vector& direction, const Ramp& ramp, float gravity, float airAccelerate)
{
    const float c1 = 1e-4;
    const float c2 = 0.9;
    const float alpha_max = 1.0;
    const float alpha_min = 0.0;
    const float tau = 0.5;
    float alpha = 1.0;
    int max_iterations = 100;

    for (int i = 0; i < max_iterations; ++i)
    {
        if (SatisfiesWolfeConditions(point, direction, alpha, ramp, gravity, airAccelerate, c1, c2))
        {
            return std::make_pair(alpha, true);
        }

        float alpha_new;
        if (i == 0)
        {
            alpha_new = alpha * tau;
        }
        else
        {
            alpha_new = (alpha_min + alpha_max) / 2;
        }

        if (SatisfiesWolfeConditions(point, direction, alpha_new, ramp, gravity, airAccelerate, c1, c2))
        {
            alpha_min = alpha_new;
        }
        else
        {
            alpha_max = alpha_new;
        }

        alpha = alpha_new;
    }

    return std::make_pair(alpha, false);
}

float Optimization::CalculatePathCost(const Vector& point, const Ramp& ramp, float gravity, float airAccelerate)
{
    const float distanceWeight = 1.0f;
    const float curvatureWeight = 0.5f;

    Vector closestPoint = GetClosestPointOnRamp(point, ramp);
    float distanceCost = (point - closestPoint).Length();

    Vector prevPoint = point - Vector(0.1f, 0.0f, 0.0f);
    Vector nextPoint = point + Vector(0.1f, 0.0f, 0.0f);

    Vector prevVector = prevPoint - point;
    Vector nextVector = nextPoint - point;

    float curvatureCost = 1.0f - MathUtils::SIMDDotProduct(prevVector.Normalized(), nextVector.Normalized());

    return distanceWeight * distanceCost + curvatureWeight * curvatureCost;
}

Vector Optimization::GetClosestPointOnRamp(const Vector& point, const Ramp& ramp)
{
    Vector rampPlaneNormal = ramp.normal.Normalized();
    Vector pointToRampStart = ramp.startPoint - point;

    float distanceToPlane = MathUtils::SIMDDotProduct(pointToRampStart, rampPlaneNormal);
    Vector closestPoint = point + rampPlaneNormal * distanceToPlane;

    return closestPoint;
}

void Optimization::UpdateInverseHessian(Matrix& inverseHessian, const Vector& direction, const Vector& gradientDifference)
{
    Matrix hessianUpdate = Matrix::OuterProduct(direction, direction) / direction.Dot(gradientDifference);
    Matrix gradientDifferenceMatrix = Matrix::OuterProduct(gradientDifference, gradientDifference);

    inverseHessian = (Matrix::Identity(3) - hessianUpdate) * inverseHessian * (Matrix::Identity(3) - hessianUpdate)
                     + hessianUpdate * (1.0f / gradientDifference.Dot(direction));
}

// Pathfinding methods

std::optional<std::vector<Vector>> Pathfinding::Pathfinding(const Vector& start, const Vector& goal, const std::vector<Ramp>& ramps)
{
    struct Node
    {
        Vector position;
        float g;
        float h;
        Node* parent;

        Node(const Vector& pos)
            : position(pos)
            , g(0.0f)
            , h(0.0f)
            , parent(nullptr)
        {}

        float F() const
        {
            return g + h;
        }
    };

    auto Heuristic = [](const Vector& a, const Vector& b)
    {
        return (a - b).Length();
    };

    std::vector<Node*> openList;
    std::unordered_set<Node*> closedList;

    Node* startNode = new Node(start);
    openList.push_back(startNode);

    while (!openList.empty())
    {
        auto it = std::min_element(openList.begin(), openList.end(), [](const Node* a, const Node* b)
        {
            return a->F() < b->F();
        });

        Node* currentNode = *it;
        openList.erase(it);
        closedList.insert(currentNode);

        if (currentNode->position == goal)
        {
            std::vector<Vector> path;
            while (currentNode != nullptr)
            {
                path.push_back(currentNode->position);
                currentNode = currentNode->parent;
            }
            std::reverse(path.begin(), path.end());

            for (Node* node : openList)
            {
                delete node;
            }
            for (Node* node : closedList)
            {
                delete node;
            }

            return path;
        }

        for (const Ramp& ramp : ramps)
        {
            Vector rampEnd = ramp.endPoint;
            if (std::find_if(closedList.begin(), closedList.end(), [&](const Node* node)
            {
                return node->position == rampEnd;
            }) == closedList.end())
            {
                float g = currentNode->g + Heuristic(currentNode->position, rampEnd);
                Node* neighbor = nullptr;
                auto it = std::find_if(openList.begin(), openList.end(), [&](const Node* node)
                {
                    return node->position == rampEnd;
                });
                if (it != openList.end())
                {
                    neighbor = *it;
                    if (g < neighbor->g)
                    {
                        neighbor->g = g;
                        neighbor->parent = currentNode;
                    }
                }
                else
                {
                    neighbor = new Node(rampEnd);
                    neighbor->g = g;
                    neighbor->h = Heuristic(rampEnd, goal);
                    neighbor->parent = currentNode;
                    openList.push_back(neighbor);
                }
            }
        }
    }

    for (Node* node : openList)
    {
        delete node;
    }
    for (Node* node : closedList)
    {
        delete node;
    }

    return std::nullopt;
}

// Utility methods

Ramp GetRampFromEntity(CBaseEntity* pEntity)
{
    if (!pEntity)
    {
        return {};
    }

    {
        std::lock_guard<std::mutex> lock(g_RampCacheMutex);
        auto it = g_RampCache.find(pEntity);
        if (it != g_RampCache.end())
        {
            return it->second;
        }
    }

    Vector startPoint = pEntity->GetAbsOrigin();
    Vector endPoint = startPoint + pEntity->GetForwardVector() * pEntity->GetValueForKey("length");
    Vector normal = pEntity->GetUpVector();
    float length = pEntity->GetValueForKey("length");
    float width = pEntity->GetValueForKey("width");

    Vector minExtents = startPoint - Vector(width / 2.0f, width / 2.0f, 0.0f);
    Vector maxExtents = endPoint + Vector(width / 2.0f, width / 2.0f, length);

    Ramp ramp;
    ramp.startPoint = startPoint;
    ramp.endPoint = endPoint;
    ramp.normal = normal;
    ramp.length = length;
    ramp.width = width;
    ramp.minExtents = minExtents;
    ramp.maxExtents = maxExtents;

    if (ramp.startPoint.Length() > 0 && ramp.endPoint.Length() > 0)
    {
        std::lock_guard<std::mutex> lock(g_RampCacheMutex);
        g_RampCache[pEntity] = ramp;
    }

    return ramp;
}

// Vector methods

Vector::Vector()
    : x(0.0f)
    , y(0.0f)
    , z(0.0f)
    , v(0.0f)
    , w(0.0f)
    , u(0.0f)
{}

Vector::Vector(float _x, float _y, float _z)
    : x(_x)
    , y(_y)
    , z(_z)
    , v(0.0f)
    , w(0.0f)
    , u(0.0f)
{}

Vector::Vector(float _x, float _y, float _z, float _v, float _w, float _u)
    : x(_x)
    , y(_y)
    , z(_z)
    , v(_v)
    , w(_w)
    , u(_u)
{}

float Vector::LengthSqr() const
{
    return x * x + y * y + z * z;
}

float Vector::Length() const
{
    return std::sqrt(LengthSqr());
}

Vector Vector::Normalized() const
{
    return MathUtils::SIMDNormalized(*this);
}

Vector Vector::Cross(const Vector& other) const
{
    return MathUtils::SIMDCrossProduct(*this, other);
}

float Vector::Dot(const Vector& other) const
{
    return MathUtils::SIMDDotProduct(*this, other);
}

Vector Vector::operator+(const Vector& other) const
{
    return Vector(x + other.x, y + other.y, z + other.z);
}

Vector Vector::operator-(const Vector& other) const
{
    return Vector(x - other.x, y - other.y, z - other.z);
}

Vector Vector::operator*(float scalar) const
{
    return Vector(x * scalar, y * scalar, z * scalar);
}

Vector Vector::operator/(float scalar) const
{
    float invScalar = 1.0f / scalar;
    return Vector(x * invScalar, y * invScalar, z * invScalar);
}

bool Vector::operator==(const Vector& other) const
{
    return x == other.x && y == other.y && z == other.z;
}

bool Vector::operator!=(const Vector& other) const
{
    return !(*this == other);
}

// Ramp methods

bool Ramp::operator==(const Ramp& other) const
{
    return startPoint == other.startPoint && endPoint == other.endPoint && normal == other.normal &&
           length == other.length && width == other.width &&
           minExtents == other.minExtents && maxExtents == other.maxExtents;
}

// Matrix methods

Matrix::Matrix(int rows, int cols)
    : m_rows(rows)
    , m_cols(cols)
    , m_data(new float[rows * cols])
{}

Matrix::Matrix(const Matrix& other)
    : m_rows(other.m_rows)
    , m_cols(other.m_cols)
    , m_data(new float[m_rows * m_cols])
{
    std::memcpy(m_data, other.m_data, m_rows * m_cols * sizeof(float));
}

Matrix::~Matrix()
{
    delete[] m_data;
}

Matrix Matrix::Identity(int size)
{
    Matrix result(size, size);
    for (int i = 0; i < size; ++i)
    {
        result.m_data[i * size + i] = 1.0f;
    }
    return result;
}

Matrix Matrix::OuterProduct(const Vector& u, const Vector& v)
{
    Matrix result(3, 3);
    result.m_data[0] = u.x * v.x;
    result.m_data[1] = u.x * v.y;
    result.m_data[2] = u.x * v.z;
    result.m_data[3] = u.y * v.x;
    result.m_data[4] = u.y * v.y;
    result.m_data[5] = u.y * v.z;
    result.m_data[6] = u.z * v.x;
    result.m_data[7] = u.z * v.y;
    result.m_data[8] = u.z * v.z;
    return result;
}

Matrix& Matrix::operator=(const Matrix& other)
{
    if (this != &other)
    {
        delete[] m_data;
        m_rows = other.m_rows;
        m_cols = other.m_cols;
        m_data = new float[m_rows * m_cols];
        std::memcpy(m_data, other.m_data, m_rows * m_cols * sizeof(float));
    }
    return *this;
}

Matrix Matrix::operator+(const Matrix& other) const
{
    Matrix result(m_rows, m_cols);
    for (int i = 0; i < m_rows * m_cols; ++i)
    {
        result.m_data[i] = m_data[i] + other.m_data[i];
    }
    return result;
}

Matrix Matrix::operator-(const Matrix& other) const
{
    Matrix result(m_rows, m_cols);
    for (int i = 0; i < m_rows * m_cols; ++i)
    {
        result.m_data[i] = m_data[i] - other.m_data[i];
    }
    return result;
}

Matrix Matrix::operator*(const Matrix& other) const
{
    Matrix result(m_rows, other.m_cols);
    for (int i = 0; i < m_rows; ++i)
    {
        for (int j = 0; j < other.m_cols; ++j)
        {
            float sum = 0.0f;
            for (int k = 0; k < m_cols; ++k)
            {
                sum += m_data[i * m_cols + k] * other.m_data[k * other.m_cols + j];
            }
            result.m_data[i * other.m_cols + j] = sum;
        }
    }
    return result;
}

Vector Matrix::operator*(const Vector& v) const
{
    Vector result;
    result.x = m_data[0] * v.x + m_data[1] * v.y + m_data[2] * v.z;
    result.y = m_data[3] * v.x + m_data[4] * v.y + m_data[5] * v.z;
    result.z = m_data[6] * v.x + m_data[7] * v.y + m_data[8] * v.z;
    return result;
}

int Matrix::Rows() const
{
    return m_rows;
}

int Matrix::Cols() const
{
    return m_cols;
}

// CBaseEntity methods

CBaseEntity::CBaseEntity()
    : m_KeyValues(nullptr)
{}

CBaseEntity::~CBaseEntity()
{
    if (m_KeyValues)
    {
        m_KeyValues->deleteThis();
    }
}

void CBaseEntity::SetKeyValues(KeyValues* kvData)
{
    if (m_KeyValues)
    {
        m_KeyValues->deleteThis();
    }
    m_KeyValues = kvData;
}

KeyValues* CBaseEntity::GetKeyValues() const
{
    return m_KeyValues;
}

void CBaseEntity::SetAbsAngles(const QAngle& angles)
{
    m_AbsAngles = angles;
}

QAngle CBaseEntity::GetAbsAngles() const
{
    return m_AbsAngles;
}

Vector CBaseEntity::GetAbsOrigin() const
{
    return m_AbsOrigin;
}

void CBaseEntity::SetAbsOrigin(const Vector& origin)
{
    m_AbsOrigin = origin;
}

Vector CBaseEntity::GetForwardVector() const
{
    Vector forward;
    AngleVectors(m_AbsAngles, &forward);
    return forward;
}

Vector CBaseEntity::GetUpVector() const
{
    Vector up;
    AngleVectors(m_AbsAngles, nullptr, nullptr, &up);
    return up;
}

float CBaseEntity::GetValueForKey(const char* key) const
{
    KeyValues* kvData = GetKeyValues();
    if (kvData)
    {
        return kvData->GetFloat(key, 0.0f);
    }
    return 0.0f;
}
