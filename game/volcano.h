#ifndef LD38_VOLCANO_VOLCANO_H
#define LD38_VOLCANO_VOLCANO_H
#include <memory>
#include <functional>
#include <SDL2/SDL.h>

#include "Box2D/Box2D.h"
#include "game/render.h"
#include "game/ParticleEmitter.h"

namespace ld38 {

class ParticleLifetimeRandomizer: public EmittedParticleCallback {
  public:
    ParticleLifetimeRandomizer()
      : ParticleLifetimeRandomizer(0.0f, 0.0f) {}
    ParticleLifetimeRandomizer(float min, float max)
      : min_lifetime_(min), max_lifetime_(max) {}
    virtual ~ParticleLifetimeRandomizer() {}
    void ParticleCreated(b2ParticleSystem * const system,
                                 int32 index) override {
        system->SetParticleLifetime(index, min_lifetime_);
    }
    void Set(float min, float max) {
        min_lifetime_ = min;
        max_lifetime_ = max;
    }
  private:
    float min_lifetime_;
    float max_lifetime_;
};

class ParticleTracker : public b2DestructionListener {
  public:
    ParticleTracker()
      : world_(nullptr), system_(nullptr) {}
    virtual ~ParticleTracker() {
        if (world_)
            world_->SetDestructionListener(nullptr);
    }
    void Init(b2World* world, b2ParticleSystem* system) {
        world_ = world;
        system_ = system;
        world_->SetDestructionListener(this);
    }

    void set_callback(
            std::function<void(b2ParticleSystem* system, int32_t index)> cb) {
        cb_ = cb;
    }

    void SayGoodbye(b2Joint*) override {}
    void SayGoodbye(b2Fixture*) override {}
    void SayGoodbye(b2ParticleGroup*) override {}
    void SayGoodbye(b2ParticleSystem* system, int32_t index) {
        if (cb_) cb_(system, index);
    }
  private:
    b2World* world_;
    b2ParticleSystem* system_;
    std::function<void(b2ParticleSystem* system, int32_t index)> cb_;
};

class RayCastAdaptor : public b2RayCastCallback {
  public:
    using Callback = std::function<float(b2Fixture*, const b2Vec2&, const b2Vec2&, float)>;
    RayCastAdaptor() {}
    RayCastAdaptor(Callback cb) : cb_(cb) {}
    virtual ~RayCastAdaptor() {}
    void set_callback(Callback cb) { cb_ = cb; }
    float ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                        const b2Vec2& normal, float fraction) override {
        return cb_(fixture, point, normal, fraction);
    }
  private:
    Callback cb_;
};

class AABBAdaptor : public b2QueryCallback {
  public:
    using Callback = std::function<bool(b2Fixture*)>;
    AABBAdaptor() {}
    AABBAdaptor(Callback cb) : cb_(cb) {}
    virtual ~AABBAdaptor() {}
    void set_callback(Callback cb) { cb_ = cb; }
    bool ReportFixture(b2Fixture* fixture) override {
        return cb_(fixture);
    }
  private:
    Callback cb_;
};

class Volcano {
  public:
    Volcano() {}
    void Init();
    void MakePyramid(int pheight);

    void Step(double deltaT);
    void HandleEvent(SDL_Event* event);
    void Coalesce(const b2Vec2& center, float radius);
  private:
    void SetupKillfield();
    void KillPolygons();
    void MakeNewGround();
    void MakeNewShape(const b2Vec2& pos, int n);
    void MakeNewPolygonFromPoints(
        const b2Vec2& center, float radius, int n, const std::vector<b2Vec2>& points);
    b2Vec2 GetClosestPoint(const b2Vec2& target,
        const std::vector<b2Vec2>& points, float* retdistance=nullptr);
    b2Vec2 GetFarthestPoint(const b2Vec2& target,
        const std::vector<b2Vec2>& points, float *retdistance=nullptr);
    void MakeNewSquareFromPoints(
        const b2Vec2& center, const std::vector<b2Vec2>& points);
    bool MakeConvexHullFromPoints(
        const b2Vec2& center, const std::vector<b2Vec2>& points);

    std::unique_ptr<b2World> world_;
    DebugDraw debug_draw_;
    b2Body* ground_;
    b2ParticleSystem* particle_system_;

    std::vector<b2Vec2> creation_list_;

    b2PolygonShape killfield_shape_;
    b2Transform killfield_transform_;

    RadialEmitter emitter_;
    ParticleLifetimeRandomizer lifetime_;
    ParticleTracker tracker_;
};



}  // namespace ld38

#endif // LD38_VOLCANO_VOLCANO_H
