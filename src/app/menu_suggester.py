#!/usr/bin/env python3

import json
import os

recipe_file = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../models/recipe_database.json")
)

try:
    with open(recipe_file) as f:
        recipes = json.load(f)
except Exception as e:
    print(f"[menu_suggester] Failed to load recipe database: {e}")
    recipes = []

def suggest_menus(available_ingredients):
    scored = []
    for recipe in recipes:
        matches = sum(1 for ing in recipe["ingredients"] if ing in available_ingredients)
        scored.append((matches, recipe))
    scored.sort(reverse=True, key=lambda x: x[0])
    return [r for score, r in scored if score > 0][:3]


