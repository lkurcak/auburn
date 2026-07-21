use auburn2d::prelude::*;
use bevy::prelude::*;

fn main() {
    App::new()
        .add_systems(Startup, setup)
        .add_systems(Update, print_names)
        .run();
}

#[derive(Component)]
pub struct CharacterCollider(pub Ball);

fn setup(mut commands: Commands) {
    commands.spawn((
        Name::new("Object One".to_string()),
        CharacterCollider(Ball::new(1.0)),
    ));
    commands.spawn((
        Name::new("Object Two".to_string()),
        CharacterCollider(Ball::new(1.0)),
    ));
}

fn print_names(query: Query<&Name>) {
    for name in &query {
        println!("{}", name);
    }
}

fn collide_among_characters(mut q_chars: Query<(&CharacterCollider, &mut Transform)>) {
    let mut combinations = q_chars.iter_combinations_mut::<2>();
    while let Some([(a_col, mut a_t), (b_col, mut b_t)]) = combinations.fetch_next() {
        let a = a_col.0.at(&*a_t);
        let b = b_col.0.at(&*b_t);

        if let Some(penetration) = a.penetrates(&b) {
            a_t.translation.x += 0.5 * penetration.x;
            a_t.translation.y += 0.5 * penetration.y;
            b_t.translation.x -= 0.5 * penetration.x;
            b_t.translation.y -= 0.5 * penetration.y;
        }
    }
}
