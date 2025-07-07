import json
import os

# load known ingredients ONCE from your recipe file
recipe_file = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../models/recipe_database.json")
)
with open(recipe_file) as f:
    recipes = json.load(f)

# collect all unique ingredients
KNOWN_INGREDIENTS = set()
for recipe in recipes:
    KNOWN_INGREDIENTS.update(recipe["ingredients"])

def parse_ingredients(text):
    found = []
    lowered_text = text.lower()
    for ingredient in KNOWN_INGREDIENTS:
        if ingredient in lowered_text:
            found.append(ingredient)
    return found
