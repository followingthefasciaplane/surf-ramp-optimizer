#pragma once

#include <ISDKTools.h>
#include <iplayerinfo.h>
#include <IForwardSys.h>
#include <vector>
#include <deque>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <IKeyValues.h>
#include <cstring>
#include <cmath>

struct Vector {
    float x, y, z, v, w, u;

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

struct Ramp {
    Vector startPoint;
    Vector endPoint;
    Vector normal;
    float length;
    float width;
    Vector minExtents;
    Vector maxExtents;

    bool operator==(const Ramp& other) const;
};

class Matrix {
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
    int m_rows, m_cols;
    float* m_data;
};

enum class OptimizationMethod {
    GradientDescent,
    ConjugateGradient,
    BFGS
};

class CBaseEntity {
public:
    CBaseEntity();
    ~CBaseEntity();

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
    Vector m_AbsOrigin;
    QAngle m_AbsAngles;
    KeyValues* m_KeyValues;
};

class CBrachistochroneOptimizer;

class PlayerMovementTracking {
public:
    PlayerMovementTracking(float maxDeviationDistance, size_t bufferCapacity = 100);
    ~PlayerMovementTracking();

    void UpdatePlayerState(const Vector& position, const Vector& velocity, const Vector& acceleration, float tickInterval);
    Vector EstimatePlayerPosition(float timeStep) const;
    bool HasDeviatedFromPath(const std::vector<Vector>& path) const;
    void RecalibratePath(const std::vector<Vector>& currentPath, const std::vector<Ramp>& ramps, CBrachistochroneOptimizer& optimizer, float tickInterval);

private:
    float m_maxDeviationDistance;
    std::deque<Vector> m_positions;
    std::deque<Vector> m_velocities;
    std::deque<Vector> m_accelerations;
    std::deque<float> m_timeIntervals;
};

class BVHNode {
public:
    Vector minExtents;
    Vector maxExtents;
    std::vector<Ramp*> objects;
    BVHNode* left;
    BVHNode* right;

    BVHNode(const Vector& min, const Vector& max);
    ~BVHNode();

    void Insert(Ramp* ramp);
    std::vector<Ramp*> Query(const Vector& point);
};

class CBrachistochroneOptimizer {
public:
    CBrachistochroneOptimizer(const Ramp& ramp, CBaseEntity* player);
    ~CBrachistochroneOptimizer();

    std::vector<Vector> Optimize(const Vector& startPos, const Vector& startVel, float tickInterval);
    void Update(const Vector& playerPos, const Vector& playerVel, const Vector& playerAccel, float tickInterval);

    const Ramp& GetRamp() const;
    const std::vector<Vector>& GetPath() const;
    CBaseEntity* GetPlayer() const;

private:
    bool IsPathValid(const Vector& point) const;

    Ramp m_Ramp;
    CBaseEntity* m_Player;
    float m_AirAccelerate;
    float m_Gravity;
    std::vector<Vector> m_OptimizedPath;
    std::unique_ptr<PlayerMovementTracking> m_PlayerTracker;

    friend class PlayerMovementTracking;
};

class SurfRampOptimizer : public IEntityListener {
public:
    bool SDK_OnLoad(char* error, size_t maxlength, bool late);
    void SDK_OnUnload();
    void SDK_OnAllLoaded();
    bool QueryRunning(char* error, size_t maxlength);

    void OnEntityCreated(CBaseEntity* pEntity, const char* classname) override;
    void OnEntityDestroyed(CBaseEntity* pEntity) override;

    bool SDK_OnMetamodLoad(ISmmAPI* ismm, char* error, size_t maxlen, bool late);

private:
    bool SetupGameInterfaces(char* error, size_t maxlength);
    bool LoadConfigurations(char* error, size_t maxlength);
    void InitializeDataStructures();
};

extern SurfRampOptimizer g_SurfRampOptimizer;
extern IForward* g_fwdOnStartTouchRamp;
extern std::vector<std::unique_ptr<CBrachistochroneOptimizer>> g_SurfRamps;

void FireOnStartTouchRampForward(CBaseEntity* pPlayer, CBaseEntity* pRamp);
Ramp GetRampFromEntity(CBaseEntity* pEntity);
void Optimize(const CBasePlayer& pPlayer, const Ramp& ramp);
void Predict(const CBasePlayer& pPlayer);

namespace CollisionDetection {
    void Initialize();
    void Shutdown();
    void InsertRamp(Ramp* ramp);
    bool Intersects(const Vector& point, const Ramp& ramp);
    bool GJKIntersection(const Ramp& ramp1, const Ramp& ramp2);
    Vector SupportPoint(const Ramp& ramp1, const Ramp& ramp2, const Vector& direction);
    Vector FarthestPointInDirection(const Ramp& ramp, const Vector& direction);
    bool UpdateSimplex(Vector* simplex, Vector& direction, int index);
}

namespace NumericalMethods {
    Vector SolveBrachistochrone(const Vector& startPos, const Vector& endPos, float time, float gravity, float airAccelerate);
    Vector SolveBrachistochroneAdaptive(const Vector& startPos, const Vector& endPos, float time, float gravity, float airAccelerate, float errorTolerance);
    float CalculateBrachistochronePathTime(const Vector& startPos, const Vector& endPos, float gravity);
    Vector CalculateGradient(const Vector& point, const Ramp& ramp, float gravity, float airAccelerate);

    float GaussianQuadrature(float a, float b, int n, float (*f)(float, void*), void* params);
    Vector DormandPrince54(float t0, float tf, const Vector& y0, Vector (*f)(float, Vector, void*), void* params, float& h);
}

namespace MathUtils {
    Vector CrossProduct(const Vector& a, const Vector& b);
    float DotProduct(const Vector& a, const Vector& b);
    float Length(const Vector& v);
    Vector Normalized(const Vector& v);

    Vector SIMDCrossProduct(const Vector& a, const Vector& b);
    float SIMDDotProduct(const Vector& a, const Vector& b);
    float SIMDLength(const Vector& v);
    Vector SIMDNormalized(const Vector& v);

    float AdaptiveGaussKronrod(float a, float b, float tolerance, int maxDepth, float (*f)(float, void*), void* params);
}

namespace Optimization {
    std::vector<Vector> OptimizePath(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate);
    std::vector<Vector> OptimizePathConjugateGradient(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate);
    std::vector<Vector> OptimizePathBFGS(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate);
    bool SatisfiesWolfeConditions(const Vector& point, const Vector& direction, float alpha, const Ramp& ramp, float gravity, float airAccelerate, float c1, float c2);
    float LineSearch(const Vector& point, const Vector& direction, const Ramp& ramp, float gravity, float airAccelerate);
    float CalculatePathCost(const Vector& point, const Ramp& ramp, float gravity, float airAccelerate);
    Vector GetClosestPointOnRamp(const Vector& point, const Ramp& ramp);
    void UpdateInverseHessian(Matrix& inverseHessian, const Vector& direction, const Vector& gradientDifference);
}

namespace Pathfinding {
    std::vector<Vector> Pathfinding(const Vector& start, const Vector& goal, const std::vector<Ramp>& ramps);
}