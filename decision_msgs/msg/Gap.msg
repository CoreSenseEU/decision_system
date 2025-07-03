# A unique identifier for the decision actively being solved.
# string id

# NOTE: This is called CSTemplate to avoid name collision with the c++ keyword `template`
#
# A set of CoreSense templates making up the difference between the desired and
# current understanding states. Upon a successful decision, all templates will
# be matched by at least one alternative modelet. Unless specified in the
# template requirements, one alternative may match multiple templates.
CSTemplate[] templates

