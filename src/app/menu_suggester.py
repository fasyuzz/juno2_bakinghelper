import json
import os

def suggest_menus(available_ingredients):
    recipe_file = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../models/recipe_database.json")
    )
    try:
        with open(recipe_file) as f:
            recipes = json.load(f)
    except Exception as e:
        print(f"Failed to load recipe database: {e}")
        return []

    scored = []
    for recipe in recipes:
        matches = sum(1 for ing in recipe["ingredients"] if ing in available_ingredients)
        scored.append((matches, recipe))

    # sort best matches first
    scored.sort(reverse=True, key=lambda x: x[0])

    # pick top 3 with at least 1 match
    suggestions = [r for score, r in scored if score > 0][:3]
    return suggestions
