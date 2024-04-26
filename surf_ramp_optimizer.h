#pragma once

#include <vector>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <optional>

// Forward declarations
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

// Vector class
class Vector
{
public:
    float x, y, z;
    float v, w, u;

    Vector();
    Vector(float _x, float _y, float _z);
    Vector(float _x, float _y, float _z, float _v, float _w, float _u);

    float LengthSqr() const;
    float Length() const;
    Vector Normalized() const;
    Vector Cross(const Vector& other) const;
    float Dot(const Vector& other) const;

    Vector operator+(const Vector& other) const;
    Vector operator-(const Vector& other) const;
    Vector operator*(float scalar) const;
    Vector operator/(float scalar) const;
    bool operator==(const Vector& other) const;
    bool operator!=(const Vector& other) const;
};

// Ramp struct
struct Ramp
{
    Vector startPoint;
    Vector endPoint;
    Vector normal;
    float length;
    float width;
    Vector minExtents;
    Vector maxExtents;

    bool operator==(const Ramp& other) const;
};

// Matrix class
class Matrix
{
public:
    Matrix(int rows, int cols);
    Matrix(const Matrix& other);
    ~Matrix();

    static Matrix Identity(int size);
    static Matrix OuterProduct(const Vector& u, const Vector& v);

    Matrix& operator=(const Matrix& other);
    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    Vector operator*(const Vector& v) const;

    int Rows() const;
    int Cols() const;

private:
    int m_rows;
    int m_cols;
    float* m_data;
};

// CBaseEntity class
class CBaseEntity
{
public:
    CBaseEntity();
    virtual ~CBaseEntity();

    void SetKeyValues(KeyValues* kvData);
    KeyValues* GetKeyValues() const;
    void SetAbsAngles(const QAngle& angles);
    QAngle GetAbsAngles() const;
    Vector GetAbsOrigin() const;
    void SetAbsOrigin(const Vector& origin);
    Vector GetForwardVector() const;
    Vector GetUpVector() const;
    float GetValueForKey(const char* key) const;

private:
    KeyValues* m_KeyValues;
    QAngle m_AbsAngles;
    Vector m_AbsOrigin;
};

// PlayerMovementTracking class
class PlayerMovementTracking
{
public:
    PlayerMovementTracking(float maxDeviationDistance, size_t bufferCapacity = 1000);

    void UpdatePlayerState(const Vector& position, const Vector& velocity, const Vector& acceleration, float tickInterval);
    Vector EstimatePlayerPosition(float timeStep) const;
    bool HasDeviatedFromPath(const std::vector<Vector>& path) const;
    void RecalibratePath(const std::vector<Vector>& currentPath, const std::vector<Ramp>& ramps, CBrachistochroneOptimizer& optimizer, float tickInterval);

private:
    float m_MaxDeviationDistance;
    std::deque<Vector> m_Positions;
    std::deque<Vector> m_Velocities;
    std::deque<Vector> m_Accelerations;
    std::deque<float> m_TimeIntervals;
};

// BVHNode class
class BVHNode
{
public:
    BVHNode(const Vector& min, const Vector& max);
    ~BVHNode();

    void Insert(Ramp* ramp);
    std::vector<Ramp*> Query(const Vector& point) const;

private:
    Vector minExtents;
    Vector maxExtents;
    BVHNode* left;
    BVHNode* right;
    std::vector<Ramp*> objects;
};

// CBrachistochroneOptimizer class
class CBrachistochroneOptimizer
{
public:
    CBrachistochroneOptimizer(const Ramp& ramp, CBaseEntity* player);

    std::vector<Vector> Optimize(const Vector& startPos, const Vector& startVel, float tickInterval);
    void Update(const Vector& playerPos, const Vector& playerVel, const Vector& playerAccel, float tickInterval);
    bool IsPathValid(const Vector& point) const;

    const Ramp& GetRamp() const;
    const std::vector<Vector>& GetPath() const;
    CBaseEntity* GetPlayer() const;

    std::vector<Vector> m_OptimizedPath;

private:
    Ramp m_Ramp;
    CBaseEntity* m_Player;
    PlayerMovementTracking m_PlayerTracker;
    float m_AirAccelerate;
    float m_Gravity;
};

// SurfRampOptimizer class
class SurfRampOptimizer : public SDKExtension
{
public:
    virtual bool SDK_OnLoad(char* error, size_t maxlength, bool late) override;
    virtual void SDK_OnUnload() override;
    virtual void SDK_OnAllLoaded() override;
    virtual bool QueryRunning(char* error, size_t maxlength) override;
    virtual void OnEntityCreated(CBaseEntity* pEntity, const char* classname) override;
    virtual void OnEntityDestroyed(CBaseEntity* pEntity) override;

    static void OnStartTouch(CBaseEntity* pOther);
    bool SDK_OnMetamodLoad(ISmmAPI* ismm, char* error, size_t maxlen, bool late);

private:
    bool SetupGameInterfaces(char* error, size_t maxlength);
    bool LoadConfigurations(char* error, size_t maxlength);
    void InitializeDataStructures();
};

// Global variables
extern SurfRampOptimizer g_SurfRampOptimizer;
extern std::vector<std::unique_ptr<CBrachistochroneOptimizer>> g_SurfRamps;
extern BVHNode* g_BVHRoot;
extern std::unordered_map<CBaseEntity*, Ramp> g_RampCache;

// Utility functions
Ramp GetRampFromEntity(CBaseEntity* pEntity);
