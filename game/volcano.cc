#include <cstdlib>
#include <set>
#include <vector>

#include "game/volcano.h"
#include "Box2D/Box2D.h"
#include <gflags/gflags.h>

DEFINE_bool(coalesce, true, "Try to coalesce bodies together");

namespace ld38 {

int random(int min, int max) {
    return min + rand() % (max-min);
}

double random() {
    return double(rand()) / double(RAND_MAX);
}

double random(double min, double max) {
    return min + random() * (max-min);
}

void Volcano::Init() {
    world_.reset(new b2World(b2Vec2(0.0f, -9.8f)));
    world_->SetDebugDraw(&debug_draw_);
    debug_draw_.SetFlags(
            b2Draw::e_shapeBit |
            b2Draw::e_particleBit |
            b2Draw::e_jointBit);

    const b2ParticleSystemDef psd;
    particle_system_ = world_->CreateParticleSystem(&psd);
    particle_system_->SetGravityScale(0.4f);
    particle_system_->SetDensity(1.2f);
    particle_system_->SetRadius(0.25f);
    particle_system_->SetMaxParticleCount(5000.0);
    particle_system_->SetDestructionByAge(true);

    tracker_.Init(world_.get(), particle_system_);
    tracker_.set_callback([=](b2ParticleSystem* system, int32 index) {
        b2Vec2 pos = system->GetPositionBuffer()[index];
        creation_list_.push_back(pos);
    });

    b2BodyDef bd;
    ground_ = world_->CreateBody(&bd);

    b2PolygonShape shape;
    shape.SetAsBox(50, 1); //, b2Vec2(0.0, 0.0), 0.0);
    ground_->CreateFixture(&shape, 0.0f);

    std::vector<b2Vec2> chamber = {b2Vec2(-0.3f, 10.0f),
                                   b2Vec2(-50.0f, 10.0f),
                                   b2Vec2(-50.0f, 0.0f),
                                   b2Vec2(-5.0f, 0.0f),
                                   b2Vec2(-0.3f, 9.0f)};
    shape.Set(chamber.data(), chamber.size());
    ground_->CreateFixture(&shape, 0.0f);

    chamber = {b2Vec2(0.3f, 10.0f),
               b2Vec2(50.0f, 10.0f),
               b2Vec2(50.0f, 0.0f),
               b2Vec2(5.0f, 0.0f),
               b2Vec2(0.3f, 9.0f)};
    shape.Set(chamber.data(), chamber.size());
    ground_->CreateFixture(&shape, 0.0f);

    MakePyramid(20);

    const float faucetlen = particle_system_->GetRadius() * 4.0f;
    lifetime_.Set(30.0f, 90.0f);
    emitter_.SetParticleSystem(particle_system_);
    emitter_.SetCallback(&lifetime_);
    emitter_.SetPosition(b2Vec2(0.0f, 5.0f));
    emitter_.SetVelocity(b2Vec2(0.0f, 0.0f));
    emitter_.SetSize(b2Vec2(0.0f, faucetlen));
    emitter_.SetColor(b2ParticleColor(0xe6, 0x1f, 0x03, 255));
    emitter_.SetEmitRate(0.0f);
    emitter_.SetParticleFlags(b2_destructionListenerParticle);

    SetupKillfield();
}

void Volcano::SetupKillfield() {
    killfield_shape_.SetAsBox(200.0f, 1.0f);
    killfield_transform_.Set(b2Vec2(0, -2), 0);
}

void Volcano::KillPolygons() {
    b2AABB aabb;
    aabb.lowerBound = {-200, -10};
    aabb.upperBound = {200, -2};
    std::vector<b2Fixture*> found;
    AABBAdaptor adaptor([&](b2Fixture* f) { found.push_back(f); return true; });
    world_->QueryAABB(&adaptor, aabb);

    for(const auto f: found) {
        b2Body *b = f->GetBody();
        if (b == ground_) continue;
        b->GetWorld()->DestroyBody(b);
    }
}

void Volcano::MakeNewShape(const b2Vec2& pos, int n) {
    const double pi = 3.141592654;
    double pinc = 2.0* pi / double(n);
    double a = 0.0;
    std::vector<b2Vec2> points;
    for(int i=0; i<n; ++i, a+=pinc) {
        points.emplace_back(0.5 * std::cos(a), 0.5 * std::sin(a));
    }

    b2PolygonShape shape;
    shape.Set(points.data(), points.size());
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position = pos;
    b2Body* body = world_->CreateBody(&bd);
    body->CreateFixture(&shape, 1.0f);
}

void Volcano::MakeNewGround() {
    for(const auto& pos : creation_list_) {
        if (pos.y > 10.0) {
            int sides = 4; // + std::rand() % 5;
            MakeNewShape(pos, sides);
        }
    }
    creation_list_.clear();
}

void Volcano::MakePyramid(int pheight) {
    b2PolygonShape shape;
    shape.SetAsBox(0.45f, 0.45f);
    b2Vec2 pa(-11.0f, 10.0f);
    b2Vec2 pb;
    b2Vec2 deltaa(0.525, 1.05f);
    b2Vec2 deltab(1.05f, 0.0f);
    for(int i=0; i<pheight; i++) {
        pb = pa;
        for(int j=i; j<pheight; j++) {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position = pb;
            b2Body* body = world_->CreateBody(&bd);
            body->CreateFixture(&shape, 1.0f);
            pb += deltab;
        }
        pa += deltaa;
    }
}

b2Vec2 Volcano::GetClosestPoint(const b2Vec2& target, const std::vector<b2Vec2>& points, float *retdistance) {
    b2Vec2 closest;
    float distance = 1e9;
    for(const auto& p : points) {
        b2Vec2 diff = target - p;
        float d = std::sqrt(diff.x*diff.x + diff.y*diff.y);
        if (d < distance) {
            distance = d;
            closest = p;
        }
    }
    if (retdistance) *retdistance = distance;
    return closest;
}

b2Vec2 Volcano::GetFarthestPoint(const b2Vec2& target, const std::vector<b2Vec2>& points, float *retdistance) {
    b2Vec2 farthest;
    float distance = 0;
    for(const auto& p : points) {
        b2Vec2 diff = target - p;
        float d = std::sqrt(diff.x*diff.x + diff.y*diff.y);
        if (d > distance) {
            distance = d;
            farthest = p;
        }
    }
    if (retdistance) *retdistance = distance;
    return farthest;
}

void Volcano::MakeNewPolygonFromPoints(
    const b2Vec2& center, float radius, int n, const std::vector<b2Vec2>& points) {
    const double pi = 3.141592654;
    double pinc = 2.0* pi / double(n);
    double a = pi / 4.0;
    std::vector<b2Vec2> polygon;
    b2Vec2 last;
    for(int i=0; i<n; ++i, a+=pinc) {
        b2Vec2 k(radius * std::cos(a), radius * std::sin(a));
        b2Vec2 p = GetClosestPoint(center+k, points) - center;
        if (p != last) {
            polygon.push_back(p);
        }
        last = p;
    }
    printf("   chose %lu points\n", polygon.size());

    b2PolygonShape shape;
    shape.Set(polygon.data(), polygon.size());
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position = center;
    b2Body* body = world_->CreateBody(&bd);
    body->CreateFixture(&shape, 1.0f);
}

void Volcano::MakeNewSquareFromPoints(
    const b2Vec2& center, const std::vector<b2Vec2>& points) {
    float top = center.y;
    float left = center.x;
    float bottom = center.y;
    float right = center.x;
    for(const auto& p : points) {
        if (p.x < left) left = p.x;
        if (p.x > right) right = p.x;
        if (p.y < bottom) bottom = p.y;
        if (p.y > top) top = p.y;
    }
    std::vector<b2Vec2> polygon = {
        b2Vec2(left, top) - center,
        b2Vec2(left, bottom) - center,
        b2Vec2(right, bottom) - center,
        b2Vec2(right, top) - center};
    b2PolygonShape shape;
    shape.Set(polygon.data(), polygon.size());
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position = center;
    b2Body* body = world_->CreateBody(&bd);
    body->CreateFixture(&shape, 1.0f);
}

namespace {
float ccw(const b2Vec2& p1, const b2Vec2& p2, const b2Vec2& p3) {
    return (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.y);
}
int orientation(const b2Vec2& p, const b2Vec2& q, const b2Vec2& r) {
    float val = (q.y - p.y) * (r.x - q.x) -
                (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;
    return (val > 0) ? 1 : 2;
}
}

bool Volcano::MakeConvexHullFromPoints(
    const b2Vec2& center, const std::vector<b2Vec2>& points) {

    b2Vec2 center2;
    for(const auto& p : points) {
        center2 += p;
    }
    center2.x /= float(points.size());
    center2.y /= float(points.size());

    // The "giftwrapping" algorithm
    std::vector<b2Vec2> hull;
    unsigned leftmost = 0;
    for(unsigned i=1; i<points.size(); i++) {
        if (points[i].x < points[leftmost].x) {
            leftmost = i;
        }
    }
    unsigned p = leftmost;
    unsigned q;
    do {
        b2Vec2 k = points[p] - center2;
        hull.push_back(k);
        if (hull.size() > 9 || std::isnan(k.x) || std::isnan(k.y))
            return false;

        q = (p + 1) % points.size();
        for(unsigned i=0; i<points.size(); i++) {
            if (orientation(points[p], points[i], points[q]) == 2) {
                q = i;
            }
        }
        p = q;
    } while(p != leftmost);

    /*
    printf("Found hull of %u points\n", hull.size());
    for(const auto& p : hull) {
        printf("    %f,%f\n", p.x, p.y);
    }
    */

    b2PolygonShape shape;
    shape.Set(hull.data(), hull.size());
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position = center2;
    b2Body* body = world_->CreateBody(&bd);
    body->CreateFixture(&shape, 1.0f);
    return true;
}



void Volcano::Coalesce(const b2Vec2& center, float radius) {
    b2AABB aabb;
    aabb.lowerBound = center - b2Vec2(radius, radius);
    aabb.upperBound = center + b2Vec2(radius, radius);
    std::vector<b2Fixture*> found;
    AABBAdaptor adaptor([&](b2Fixture* f) { found.push_back(f); return true; });
    world_->QueryAABB(&adaptor, aabb);

//    printf("Query from (%f,%f) to (%f,%f)\n",
//            aabb.lowerBound.x, aabb.lowerBound.y,
//            aabb.upperBound.x, aabb.upperBound.y);

    std::vector<b2Vec2> points;
    std::vector<b2Body*> destroy;
    for(const auto f: found) {
        b2Body *b = f->GetBody();
        if (b == ground_) continue;
        if (f->GetShape()->GetType() != b2Shape::e_polygon)
            continue;
        auto* shape = static_cast<b2PolygonShape*>(f->GetShape());
        auto pos = b->GetPosition();

        for(int i=0; i < shape->GetVertexCount(); i++) {
            points.push_back(pos + shape->GetVertex(i));
        }
        destroy.push_back(b);
    }

    if (points.size()) {
        // If we found shapes, coalese into a bigger object
        float r2;
        //GetFarthestPoint(center, points, &r2);
        printf("New polygon from %lu points\n", points.size());
        if (MakeConvexHullFromPoints(center, points)) {
            for(auto& b : destroy) {
                b->GetWorld()->DestroyBody(b);
            }
        }
    }
}

void Volcano::Step(double deltaT) {
    world_->Step(deltaT, 8, 3);
    emitter_.Step(deltaT, nullptr, 0);
    particle_system_->DestroyParticlesInShape(killfield_shape_, killfield_transform_);
    KillPolygons();
    MakeNewGround();

    b2Vec2 cpoint(random(-100.0, 100.0), random(10.0, 100.0));
    double radius = random(0.5, 2.0);
    if (FLAGS_coalesce && (cpoint.x < -4 || cpoint.x > 4)) {
        Coalesce(cpoint, 2.0);
    }

    world_->DrawDebugData();
}

void Volcano::HandleEvent(SDL_Event* event) {
    switch(event->type) {
    case SDL_KEYUP:
    case SDL_KEYDOWN:
        if (event->key.keysym.scancode == SDL_SCANCODE_A) {
              emitter_.SetEmitRate(event->type == SDL_KEYDOWN ? 120.0f : 0.0f);
        }
        break;
    }
}


}  // namespace ld38
